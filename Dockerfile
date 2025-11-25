# syntax=docker/dockerfile:1.7-labs
FROM python:3.12-slim

# Speed + deterministic behavior
ENV PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Copy source code and install package
COPY . /tmp/motor_python

# Install package from local source with a cache mount
# NOTE: --root-user-action needs a value; use =ignore to silence root warnings.
RUN --mount=type=cache,target=/root/.cache/pip \
    python -m pip install --upgrade pip && \
    pip install --root-user-action=ignore /tmp/motor_python

# Simple smoke test script
WORKDIR /app
RUN printf '%s\n' \
  "import importlib, os, sys" \
  "m = importlib.import_module('motor_python')" \
  "print('✅ import ok:', getattr(m, '__version__', 'unknown'), 'on', sys.version)" \
  > smoke.py

CMD ["python", "smoke.py"]
