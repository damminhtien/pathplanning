.PHONY: install install-dev lint format precommit test typecheck

install:
	pip install -r requirements.txt

install-dev:
	pip install -r requirements-dev.txt

lint:
	ruff check .

lint-google:
	pylint --rcfile .pylintrc pathplanning Search_based_Planning Sampling_based_Planning CurvesGenerator

format:
	ruff format .

precommit:
	pre-commit run --all-files

typecheck:
	pyright

test:
	pytest -q

build:
	rm -rf dist
	python -m build

publish-test: build
	twine upload --repository testpypi dist/*

publish: build
	twine upload dist/*
