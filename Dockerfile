# syntax=docker/dockerfile:1.7-labs
FROM python:3.12-slim

# Speed + deterministic behavior
ENV PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Select package + version at build-time
ARG PKG=change_me
ARG VER=latest
ENV PKG=${PKG} VER=${VER}

# Install package from PyPI with a cache mount (huge speedup on rebuilds)
# NOTE: --root-user-action needs a value; use =ignore to silence root warnings.
RUN --mount=type=cache,target=/root/.cache/pip \
    python -m pip install --upgrade pip && \
    if [ "$VER" = "latest" ]; then \
        pip install --root-user-action=ignore "$PKG"; \
    else \
        pip install --root-user-action=ignore "$PKG==$VER"; \
    fi

# Simple smoke test script
WORKDIR /app
RUN printf '%s\n' \
  "import importlib, os, sys" \
  "m = importlib.import_module(os.environ.get('PKG', '${PKG}'))" \
  "print('✅ import ok:', getattr(m, '__version__', 'unknown'), 'on', sys.version)" \
  > smoke.py

CMD ["python", "smoke.py"]
