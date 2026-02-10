.PHONY: install install-dev lint format precommit

install:
	pip install -r requirements.txt

install-dev:
	pip install -r requirements-dev.txt

lint:
	ruff check .

format:
	ruff format .

precommit:
	pre-commit run --all-files
