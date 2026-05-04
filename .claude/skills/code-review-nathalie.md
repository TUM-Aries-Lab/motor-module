# Code Review Skill

Review the current pull request changes and provide constructive, specific feedback.
Focus on correctness, safety, readability, and adherence to our lab's coding conventions.

## Naming Conventions
- Modules: `snake_case.py`
- Classes: `PascalCase`
- Functions/methods: `snake_case`
- Constants: `UPPER_SNAKE_CASE`
- Private: prefixed with `_`
- Booleans should read as a question: `is_running`, `valid_stride`, `controller_initialized`
- **Physical quantities must include units in the name**: `angle_rad`, `velocity_rad_per_sec`, `motor_torque_nm_per_kg` — flag any variable like `angle`, `velocity`, `torque`, or `position` that is missing its unit suffix

## Type Hints
- All function signatures must have type hints for parameters and return values
- Use modern syntax: `float | None` (not `Optional[float]`), `list[str]` (not `List[str]`)
- Use `NDArray` from `numpy.typing` for numpy arrays

## Docstrings (Sphinx/reST Style)
- All public classes, methods, and functions must have docstrings
- Format: one-line summary, blank line, `:param name:`, `:return:`, `:rtype:`, `:raises:`
- Module-level docstrings at the top of every file

## Data Structures
- Prefer `@dataclass` over tuples or dicts for structured data
- Use `@dataclass(frozen=True)` for immutable config objects
- Use appropriate Enum types: `Enum` for states, `IntEnum` for numeric codes, `StrEnum` for strings
- Every enum should have a docstring

## Error Handling
- No bare `except:` — always catch specific exceptions (`OSError`, `serial.SerialException`, etc.)
- Hardware communication (I2C, serial) must be wrapped in try/except — never let it crash the control loop
- Log errors with `logger.error()` before re-raising or handling
- Safety-critical values should be clamped with `np.clip()` rather than raising exceptions

## Logging
- Use `loguru` (`from loguru import logger`), not Python's built-in `logging`
- Check appropriate log levels: `debug` for diagnostics, `info` for operations, `warning` for recoverable issues, `error` for failures, `critical` for fatal
- Log messages should include relevant context

## Threading
- Background loops should use `daemon=True` threads
- Shared data between threads must use `threading.Lock()`
- Keep critical sections (`with self.lock:`) as short as possible
- Shutdown must include `join(timeout=...)` to avoid hanging

## Constants and Configuration
- No hardcoded tunable parameters — they belong in `definitions.py`
- Groups of related constants should use frozen dataclasses
- Physical constants must have comments explaining their meaning and units

## File Handling
- Use `pathlib.Path`, not string concatenation or `os.path`
- Create directories with `mkdir(parents=True, exist_ok=True)` before writing

## Class Design
- Prefer composition over inheritance
- Use ABC + abstract methods for swappable behavior (strategy pattern)
- Use static factory methods for hardware abstraction

## Project Structure
- Source code in `src/<package_name>/` (src-layout)
- Tests in `tests/` mirroring src structure
- Data files in `data/`
- Dependencies in `pyproject.toml` (not `setup.py`)

## General
- No hardcoded values (paths, magic numbers, credentials)
- Readability matters — prefer clear variable names over comments explaining unclear names
- Prefer classes over tuples or large arrays for structured data
- Follow existing patterns in the codebase
- Use f-strings, not `.format()`
- Python 3.11+ features are welcome
