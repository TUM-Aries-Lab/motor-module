SHELL := /bin/bash

init:  # ENV SETUP
	uv sync --all-groups
	uv run pre-commit install
	@echo "Environment initialized with uv."

test:
	uv run pytest --cov=src --cov-report=term-missing --no-cov-on-fail --cov-report=xml --cov-fail-under=60
	rm .coverage

lint:
	uv run ruff format src/ tests/
	uv run ruff check --fix src/ tests/

typecheck:
	uv run pyright src

format:
	make lint
	make typecheck

clean:
	rm -rf .venv
	rm -rf .mypy_cache
	rm -rf .pytest_cache
	rm -rf build/
	rm -rf dist/
	rm -rf junit-pytest.xml
	rm -rf logs/*
	find . -name ".coverage*" -delete
	find . -name "__pycache__" -exec rm -r {} +

update:
	uv lock --upgrade

update-deep:
	uv cache clean
	make update

docker:
	docker build --no-cache -f Dockerfile -t motor_python-smoke .
	docker run --rm motor_python-smoke

app:
	sudo ./.venv/bin/python -m motor_python

tree:
	uv run python repo_tree.py --update-readme

build:
	uv build
	unzip -l dist/*.whl
	unzip -p dist/*.whl */METADATA
