"""Second-order low-pass filter with strategy pattern for integration.

Signal flow:
───────────────────────────────────────────────────────
  1. input_error       = input - filtered_output (calculate difference)
  2. damping_fb        = 2 * damping_ratio * feedback_state
  3. corrected_error   = input_error - damping_fb (apply damping correction)
  4. wn_corrected_err  = natural_freq * corrected_error
  5. feedback_state    = integral(wn_corrected_err) [INTERNAL INTERMEDIATE STATE]
  6. output_derivative = natural_freq * feedback_state [RATE OF CHANGE]
  7. filtered_output   = integral(output_derivative) [MAIN OUTPUT]
  8. filtered_output fed back to step 1 (minus input)

State equations (continuous):
  dq/dt = wn * (x - y - 2*zt*q)
  dy/dt = wn * q  =  y_dot

Transfer function:
  H(s) = wn^2 / (s^2 + 2*zt*wn*s + wn^2)

Public API
──────────
  step(x, timestamp)
      One sample at a time. Advances using the provided timestamp.
      Use this in a real-time loop where timestamps are available.

  run(x_array, timestamp_array=None)
      Whole sequence at once; resets state first.
      timestamp_array may be a matching array of timestamps, or None to use
      default timing based on cfg.time_difference.

  reset()
      Return both integrators to x0.

The solver strategy (Forward Euler / Backward Euler / Trapezoidal / RK4)
is selected via FilterDefinitions.solver_type and injected at construction.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable, Iterable
from dataclasses import dataclass

from motor_python.definitions import LowPassFilterConfig, SolverType

# Type alias: deriv_fn(q, y) -> (dq_dt, dy_dt)
DerivFn = Callable[[float, float], tuple[float, float]]


@dataclass
class LowPassFilterState:
    """Internal state tracking for the filter's two integration stages.

    :param float feedback_state:
        Intermediate state from the feedback stage.
        Satisfies d(feedback_state)/dt = wn * (input - filtered_output - 2*damping*feedback_state).
        Used to compute the output derivative and damping feedback signal.
    :param float filtered_output:
        Main filter output (the smoothed signal).
        Satisfies d(filtered_output)/dt = wn * feedback_state.
        Also fed back as negative feedback in the error calculation.

    """

    feedback_state: float = 0.0
    filtered_output: float = 0.0


class SecondOrderLowPassFilter:
    """Second-order low-pass filter with pluggable integration strategy.

    The filter owns:
        - Configuration  : FilterDefinitions (wn, zt, x0, dt, solver_type)
        - Internal state : LowPassFilterState (q, y)
        - Solver         : SolverStrategy instance (injected via solver_type)

    The solver owns nothing — it only defines HOW states advance each step.
    Swap solvers by changing cfg.solver_type; no other code changes.

    Usage — fixed timestep (real-time loop)
    ---------------------------------------
        cfg = FilterDefinitions(wn=20.0, zt=1.0, dt=0.01, solver_type=SolverType.RK4)
        lpf = SecondOrderLPF(cfg)

        y_dot, y = lpf.step(x)           # uses cfg.dt

    Usage — variable timestep (real-time loop)
    ------------------------------------------
        y_dot, y = lpf.step(x, dt=0.013)  # override dt for this step only

    Usage — whole sequence, fixed dt
    ---------------------------------
        y_array, y_dot_array = lpf.run(x_array)

    Usage — whole sequence, variable dt
    ------------------------------------
        y_array, y_dot_array = lpf.run(x_array, dt_array=timestamps_diff)
    """

    def __init__(self, config: LowPassFilterConfig) -> None:
        """Initialize the second-order low-pass filter.

        :param LowPassFilterConfig config: Configuration containing natural frequency, damping ratio, initial condition, timestep, and solver method.
        :rtype: None
        """
        self._config = config
        self._state = LowPassFilterState(
            feedback_state=config.initial_condition,
            filtered_output=config.initial_condition,
        )
        self._solver: SolverStrategy = make_solver(config.solver_type)
        self._current_input = (
            0.0  # current input, stored so derivative function can access it
        )
        self._prev_timestamp: float | None = None

    def _compute_input_error(
        self, current_input: float, filtered_output: float
    ) -> float:
        """Calculate the error between the raw input and the filtered output.

        :param float current_input: Raw filter input signal.
        :param float filtered_output: Current smoothed output from the filter.
        :return: Error signal (difference between input and output).
        :rtype: float
        """
        return current_input - filtered_output

    def _compute_damping_feedback(self, feedback_state: float) -> float:
        """Calculate the damping feedback signal proportional to the feedback state.

        :param float feedback_state: Intermediate feedback state value.
        :return: Damping feedback signal.
        :rtype: float
        """
        return 2.0 * self._config.damping_ratio * feedback_state

    def _compute_corrected_error(
        self, input_error: float, damping_feedback: float
    ) -> float:
        """Calculate the error after applying damping correction.

        :param float input_error: Error between input and output.
        :param float damping_feedback: Damping feedback signal.
        :return: Corrected error used to drive the feedback state.
        :rtype: float
        """
        return input_error - damping_feedback

    def _compute_output_derivative(self, feedback_state: float) -> float:
        """Calculate the rate of change of the filter output.

        This is proportional to the feedback state and defines how fast
        the smoothed output changes. NOT a stored state—recomputed each step.

        :param float feedback_state: Intermediate feedback state value.
        :return: Derivative of the filtered output (rate of change).
        :rtype: float
        """
        return self._config.cut_off_frequency_rad_per_sec * feedback_state

    def _deriv_fn(
        self, feedback_state: float, filtered_output: float
    ) -> tuple[float, float]:
        """Calculate the two time derivatives needed for numerical integration.

        The solver may call this multiple times per step (RK4 calls it 4x),
        each time with different trial values. The current filter input is
        stored in self._current_input, set once per step() call.

        :param float feedback_state: Trial value for intermediate feedback state.
        :param float filtered_output: Trial value for the main filter output.
        :return: Tuple of (d_feedback_state/dt, d_filtered_output/dt).
        :rtype: tuple[float, float]
        """
        input_error = self._compute_input_error(self._current_input, filtered_output)
        damping_fb = self._compute_damping_feedback(feedback_state)
        corrected_err = self._compute_corrected_error(input_error, damping_fb)
        d_feedback_dt = self._config.cut_off_frequency_rad_per_sec * corrected_err
        d_output_dt = self._compute_output_derivative(feedback_state)
        return d_feedback_dt, d_output_dt

    def step(self, x: float, time_difference: float) -> tuple[float, float]:
        """Process one input sample and return the filtered output and its rate of change.

        :param float x: Raw input signal at the current timestep.
        :param float time_difference: Time elapsed since the previous step [s].
        :return: Tuple of (filtered_output, output_derivative)—the smoothed value and its rate of change.
        :rtype: tuple[float, float]
        """
        self._current_input = x

        fb_out, output_out, fb_next, output_next = self._solver.step(
            deriv_fn=self._deriv_fn,
            feedback_state=self._state.feedback_state,
            filtered_output=self._state.filtered_output,
            time_difference=time_difference,
        )

        self._state.feedback_state = fb_next
        self._state.filtered_output = output_next

        # output_derivative is computed from the output-side feedback state value
        output_derivative = self._compute_output_derivative(fb_out)

        return output_out, output_derivative

    def run(
        self,
        x_array: Iterable[float],
        timestamp_array: Iterable[float] | None = None,
    ) -> tuple[list[float], list[float]]:
        """Process an entire input sequence and return the filtered outputs.

        Resets the filter state to the initial condition before starting, so each
        call to run() is independent and reproducible regardless of prior step() calls.

        :param Iterable[float] x_array: Input signal samples (list, numpy array, or any sequence).
        :param Iterable[float] | None timestamp_array: Optional timestamps [s]. If None, uses fixed 0.01 s intervals.
        :return: Tuple of (filtered_signal, output_derivative)—the smoothed signal and its rate of change.
        :rtype: tuple[list[float], list[float]]
        """
        self.reset()

        # Normalise inputs
        x_list = list(x_array)
        n = len(x_list)

        if timestamp_array is None:
            # Use fixed time difference of 0.01 s for all steps
            dt_list = [0.01] * n
        else:
            # Per-sample timestamps — validate length
            timestamps = list(timestamp_array)
            if len(timestamps) != n:
                raise ValueError(
                    f"timestamp_array length ({len(timestamps)}) must match x_array length ({n})."
                )
            # Check for None values
            if any(t is None for t in timestamps):
                raise ValueError("timestamp cannot be None")
            # Compute time differences: first step uses 0.01, subsequent use t - prev_t
            dt_list = [0.01]  # for first step
            for i in range(1, n):
                dt = timestamps[i] - timestamps[i - 1]
                dt_list.append(dt)

        y_array: list[float] = []
        y_dot_array: list[float] = []

        for x, dt in zip(x_list, dt_list, strict=True):
            y_dot, y = self.step(x=x, time_difference=dt)
            y_array.append(y)
            y_dot_array.append(y_dot)

        return y_array, y_dot_array

    def reset(self) -> None:
        """Reset the filter state to its initial condition.

        :rtype: None
        """
        self._state.feedback_state = self._config.initial_condition
        self._state.filtered_output = self._config.initial_condition

    @property
    def solver_name(self) -> str:
        """Get the name of the integration method being used.

        :return: Name of the active solver strategy.
        :rtype: str
        """
        return repr(self._solver)


"""Numerical integration strategies for the low-pass filter.

Design pattern: Strategy
────────────────────────
- SolverStrategy (abstract) defines the common interface.
- Each solver implements step() to advance the filter
  states over one timestep using a specific numerical method.
- The filter owns the state values; solvers only define HOW to update them.
- Solvers are interchangeable by changing the solver_type configuration.

Interface contract
──────────────────
Every solver implements:

  step(deriv_fn, feedback_state, filtered_output, dt)
    -> (fb_out, output_out, fb_next, output_next)

Where:
  deriv_fn(fb, out) -> (d_fb/dt, d_out/dt)
      Pure derivative function from the filter.
  fb_out, output_out   : current values for this step's output
  fb_next, output_next : new state values for the next step

NOTE: RK4 has no feedthrough (out == next).
      Discrete methods may have feedthrough (out may differ from next).

Solver summary
──────────────
Forward Euler   : Simple, unstable for large timesteps
Backward Euler  : More stable than Forward Euler
Trapezoidal     : Better accuracy than backward, still stable
RK4             : High accuracy, best for accurate simulation
"""


class SolverStrategy(ABC):
    """Abstract base for all integration strategies.

    The filter calls step() each timestep without knowing which solver is
    active — this is the core of the Strategy pattern.
    """

    @abstractmethod
    def step(
        self,
        deriv_fn: DerivFn,
        feedback_state: float,
        filtered_output: float,
        time_difference: float,
    ) -> tuple[float, float, float, float]:
        """Advance the two filter states by one timestep using this solver's method.

        :param DerivFn deriv_fn: Derivative function (feedback_state, filtered_output) -> (derivatives).
        :param float feedback_state: Current intermediate feedback state.
        :param float filtered_output: Current filtered output state.
        :param float time_difference: Timestep duration [s].
        :return: Tuple (q_out, y_out, q_next, y_next) with output values and next state values.
        :rtype: tuple[float, float, float, float]
        """
        ...

    def __repr__(self) -> str:
        """Get the solver name for logging and debugging.

        :return: Class name representing the solver method.
        :rtype: str
        """
        return self.__class__.__name__


class ForwardEulerSolver(SolverStrategy):
    """Simple forward Euler numerical integration method.

    Updates: new_state = old_state + dt * derivative
    Output is the old state (before update), so no feedthrough.

    Properties:
        - Simplest method, one derivative evaluation per step.
        - 1st-order accurate: error proportional to dt.
        - Can be unstable if timestep is too large.
        - Good for prototyping and simple systems.
    """

    def step(
        self,
        deriv_fn: DerivFn,
        feedback_state: float,
        filtered_output: float,
        time_difference: float,
    ) -> tuple[float, float, float, float]:
        """Advance states by one timestep using Forward Euler method.

        Output uses old state values; new states are computed for next step.

        :param DerivFn deriv_fn: Derivative function for the filter.
        :param float feedback_state: Current feedback state.
        :param float filtered_output: Current filtered output state.
        :param float time_difference: Timestep [s].
        :return: Tuple (q_out, y_out, q_next, y_next).
        :rtype: tuple[float, float, float, float]
        """
        dq_dt, dy_dt = deriv_fn(
            feedback_state, filtered_output
        )  # slope at current states

        q_out = feedback_state  # output = state BEFORE update
        y_out = filtered_output

        q_next = (
            feedback_state + time_difference * dq_dt
        )  # state advances after output is read
        y_next = filtered_output + time_difference * dy_dt

        return q_out, y_out, q_next, y_next


class BackwardEulerSolver(SolverStrategy):
    """Backward Euler numerical integration method.

    Updates: new_state = old_state + dt * derivative
    Output is the new state (after update), so there is feedthrough.

    Properties:
        - More stable than Forward Euler for stiff systems.
        - 1st-order accurate: error proportional to dt.
        - One derivative evaluation per step.
        - Better stability for larger timesteps.
    """

    def step(
        self,
        deriv_fn: DerivFn,
        feedback_state: float,
        filtered_output: float,
        time_difference: float,
    ) -> tuple[float, float, float, float]:
        """Advance states by one timestep using Backward Euler method.

        Output uses new state values (computed for this step).

        :param DerivFn deriv_fn: Derivative function for the filter.
        :param float feedback_state: Current feedback state.
        :param float filtered_output: Current filtered output state.
        :param float time_difference: Timestep [s].
        :return: Tuple (q_out, y_out, q_next, y_next).
        :rtype: tuple[float, float, float, float]
        """
        dq_dt, dy_dt = deriv_fn(
            feedback_state, filtered_output
        )  # slope at current states

        q_out = (
            feedback_state + time_difference * dq_dt
        )  # output = state + dt*u (feedthrough)
        y_out = filtered_output + time_difference * dy_dt

        q_next = q_out  # next state carries the updated value
        y_next = y_out

        return q_out, y_out, q_next, y_next


class TrapezoidalSolver(SolverStrategy):
    """Trapezoidal (Tustin / Bilinear) discrete integration.

    Approximation: 1/s ≈ T/2 * (z+1)/(z-1)

    Rule per integrator block:
        y(k)   = x(k) + dt/2 * u(k)    ← output is midpoint between current and next
        x(k+1) = y(k) + dt/2 * u(k)    ← state advances a further dt/2
               = x(k) + dt  * u(k)     ← equivalent to a full Forward Euler state step

    Properties:
        - 2nd-order accurate: better than Forward or Backward Euler.
        - Stable for larger timesteps.
        - One derivative evaluation per step.
        - Good balance between accuracy and simplicity.
    """

    def step(
        self,
        deriv_fn: DerivFn,
        feedback_state: float,
        filtered_output: float,
        time_difference: float,
    ) -> tuple[float, float, float, float]:
        """Advance states by one timestep using Trapezoidal method.

        Output and next states are computed for better accuracy.

        :param DerivFn deriv_fn: Derivative function for the filter.
        :param float feedback_state: Current feedback state.
        :param float filtered_output: Current filtered output state.
        :param float time_difference: Timestep [s].
        :return: Tuple (q_out, y_out, q_next, y_next).
        :rtype: tuple[float, float, float, float]
        """
        dq_dt, dy_dt = deriv_fn(
            feedback_state, filtered_output
        )  # slope at current states

        q_out = feedback_state + (time_difference / 2.0) * dq_dt  # output = midpoint
        y_out = filtered_output + (time_difference / 2.0) * dy_dt

        q_next = q_out + (time_difference / 2.0) * dq_dt  # = q + dt * dq_dt
        y_next = y_out + (time_difference / 2.0) * dy_dt  # = y + dt * dy_dt

        return q_out, y_out, q_next, y_next


class RK4Solver(SolverStrategy):
    """Runge-Kutta 4th order integration method.

    Uses 4 derivative samples per step for high accuracy.
    Approximates the solution curve by evaluating the derivative at
    multiple points and taking a weighted average.

    Properties:
        - Highest accuracy: error proportional to dt⁴.
        - Best for accurate continuous-time simulation.
        - Four derivative evaluations per step.
        - Very stable across a wide range of timesteps.
        - Matches Simulink's continuous-time integrators.
    """

    def step(
        self,
        deriv_fn: DerivFn,
        feedback_state: float,
        filtered_output: float,
        time_difference: float,
    ) -> tuple[float, float, float, float]:
        """Advance states by one timestep using RK4 method.

        Uses four derivative evaluations at different points for high accuracy.

        :param DerivFn deriv_fn: Derivative function for the filter.
        :param float feedback_state: Current feedback state.
        :param float filtered_output: Current filtered output state.
        :param float time_difference: Timestep [s].
        :return: Tuple (q_out, y_out, q_next, y_next).
        :rtype: tuple[float, float, float, float]
        """
        # Stage 1: derivative at current state
        k1_q, k1_y = deriv_fn(feedback_state, filtered_output)

        # Stage 2: derivative at midpoint using k1
        k2_q, k2_y = deriv_fn(
            feedback_state + 0.5 * time_difference * k1_q,
            filtered_output + 0.5 * time_difference * k1_y,
        )

        # Stage 3: derivative at midpoint using k2 (refined)
        k3_q, k3_y = deriv_fn(
            feedback_state + 0.5 * time_difference * k2_q,
            filtered_output + 0.5 * time_difference * k2_y,
        )

        # Stage 4: derivative at estimated next state
        k4_q, k4_y = deriv_fn(
            feedback_state + time_difference * k3_q,
            filtered_output + time_difference * k3_y,
        )

        # Weighted combination: midpoint estimates weighted twice
        q_next = feedback_state + (time_difference / 6.0) * (
            k1_q + 2 * k2_q + 2 * k3_q + k4_q
        )
        y_next = filtered_output + (time_difference / 6.0) * (
            k1_y + 2 * k2_y + 2 * k3_y + k4_y
        )

        return q_next, y_next, q_next, y_next


def make_solver(solver_type: SolverType) -> SolverStrategy:
    """Create a solver instance for the specified integration method.

    :param SolverType solver_type: The desired numerical integration method.
    :return: An instance of the corresponding solver strategy.
    :rtype: SolverStrategy
    :raises ValueError: If an unknown solver type is requested.
    """
    _map = {
        SolverType.FORWARD_EULER: ForwardEulerSolver,
        SolverType.BACKWARD_EULER: BackwardEulerSolver,
        SolverType.TRAPEZOIDAL: TrapezoidalSolver,
        SolverType.RUNGE_KUTTA: RK4Solver,
    }
    cls = _map.get(solver_type)
    if cls is None:
        raise ValueError(f"Unknown solver type: {solver_type}")
    return cls()
