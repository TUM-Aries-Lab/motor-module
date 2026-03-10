SHELL := /bin/bash

init:  # ENV SETUP
	uv sync --all-groups
	uv run pre-commit install
	@echo "Environment initialized with uv."

test:
	uv run pytest -m "not hardware and not hardware_uart" --cov=src --cov-report=term-missing --no-cov-on-fail --cov-report=xml --cov-fail-under=70
	rm -f .coverage

test-hardware: test-hardware-can  ## Alias for test-hardware-can (CAN is the active setup)

test-hardware-can:  ## Run only CAN hardware tests (requires motor on can0, NO UART cable)
	sudo bash setup_can.sh
	uv run pytest tests/hardware_can_test.py -v --cov=src --cov-report=term-missing --no-cov-on-fail
	rm -f .coverage

test-hardware-uart:  ## Run only UART hardware tests (requires R-Link cable; UART blocks CAN — never run alongside CAN)
	uv run pytest -m hardware_uart -v --cov=src --cov-report=term-missing --no-cov-on-fail
	rm -f .coverage

test-hardware-all:  ## Run ALL hardware tests (requires both CAN motor and UART serial connected)
	uv run pytest tests/hardware_can_test.py tests/hardware_test.py -v --cov=src --cov-report=term-missing --no-cov-on-fail
	rm -f .coverage

setup-can:  ## Configure can0 interface (requires sudo)
	sudo bash setup_can.sh

install-can-sudoers:  ## Grant passwordless sudo for CAN commands (run once; needed for hardware tests)
	@echo "Installing /etc/sudoers.d/can-setup ..."
	@printf '%s ALL=(ALL) NOPASSWD: /usr/sbin/ip, /usr/sbin/modprobe, /bin/bash /home/%s/lower_exosuit/motor/motor-module/setup_can.sh\n' "$$USER" "$$USER" | sudo tee /etc/sudoers.d/can-setup > /dev/null
	sudo chmod 0440 /etc/sudoers.d/can-setup
	sudo visudo -c -f /etc/sudoers.d/can-setup
	@echo "Done. sudo -n now works for CAN interface commands."

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
	uv run python -m motor_python

tree:
	uv run python repo_tree.py --update-readme

build:
	uv build
	unzip -l dist/*.whl
	unzip -p dist/*.whl */METADATA
