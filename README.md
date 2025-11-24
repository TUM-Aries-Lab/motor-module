# template-python
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/template-python/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/template-python?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/template-python/actions/workflows/ci.yml/badge.svg)

Simple README.md for a Python project template.

Do ***NOT*** clone this repository. Please use it as a template instead. This readme is just here to serve as a template for you to get started faster.

## Install
To install the library run:

```bash
uv install <your-package-name>
```

OR

```bash
uv install git+https://github.com/TUM-Aries-Lab/<your-package-name>.git@<specific-tag>
```

## Development
0. Install [uv](https://docs.astral.sh/uv/getting-started/installation/)
1. Install [pyenv](https://github.com/pyenv/pyenv?tab=readme-ov-file#installation)
2. ```pyenv install <desired-python-version>  # install the required python version```
3. ```pyenv global <desired-python-version>  # set the required python version```
4. ```git clone git@github.com:TUM-Aries-Lab/template-python.git```
5. `make init` to create the virtual environment and install dependencies
6. `make format` to format the code and check for errors
7. `make test` to run the test suite
8. `make clean` to delete the temporary files and directories

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package run:

```bash
uv build
uv publish  # make sure your version in pyproject.toml is updated
```
The package can then be found at: https://pypi.org/project/change-me

## Module Usage
```python
"""Basic docstring for my module."""

from loguru import logger

from change_me.config import definitions

def main() -> None:
    """Run a simple demonstration."""
    logger.info("Hello World!")

if __name__ == "__main__":
    main()
```

## Program Usage
```bash
uv run python -m change_me
```
