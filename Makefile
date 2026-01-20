SHELL := /bin/bash

init:  # ENV SETUP
	uv sync --all-groups
	uv run pre-commit install
	@echo "Environment initialized with uv."

test:
	uv run pytest -m "not hardware" --cov=src --cov-report=term-missing --no-cov-on-fail --cov-report=xml --cov-fail-under=55
	rm .coverage

test-hardware:
	uv run pytest -m hardware --cov=src --cov-report=term-missing --no-cov-on-fail --cov-report=xml --cov-fail-under=55
	rm .coverage

test-all:
	uv run pytest --cov=src --cov-report=term-missing --no-cov-on-fail --cov-report=xml --cov-fail-under=55
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

install-can:
	uv pip install spidev
	@echo "CAN dependencies (spidev) installed."

docker:
	docker build --no-cache -f Dockerfile -t motor_python-smoke .
	docker run --rm motor_python-smoke

app:
	@echo "=========================================="
	@echo "  Motor Control Interface Selection"
	@echo "=========================================="
	@echo "1) UART (RS485/Serial)"
	@echo "2) CAN (MCP2515 SPI-CAN) - Motor ID: 0x68"
	@echo ""
	@read -p "Select interface [1-2]: " choice; \
	case $$choice in \
		1) \
			echo ""; \
			echo "Starting UART interface..."; \
			uv run python -m motor_python --interface uart ;; \
		2) \
			echo ""; \
			echo "Starting CAN interface (Motor ID: 0x68)..."; \
			uv run python -m motor_python --interface can --motor-id 0x68 ;; \
		*) \
			echo ""; \
			echo "Invalid choice. Defaulting to UART..."; \
			uv run python -m motor_python --interface uart ;; \
	esac

tree:
	uv run python repo_tree.py --update-readme

build:
	uv build
	unzip -l dist/*.whl
	unzip -p dist/*.whl */METADATA
