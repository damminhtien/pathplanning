.PHONY: install install-dev lint format precommit test

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

test:
	pytest -q

build:
	rm -rf dist
	python -m build

publish-test: build
	twine upload --repository testpypi dist/*

publish: build
	twine upload dist/*
