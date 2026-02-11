.PHONY: install install-dev lint format precommit test typecheck benchmark

LINT_PATHS := pathplanning/api.py pathplanning/registry.py pathplanning/search2d.py pathplanning/viz tests scripts

install:
	pip install -r requirements.txt

install-dev:
	pip install -r requirements-dev.txt

lint:
	ruff check $(LINT_PATHS)

lint-google:
	pylint --rcfile .pylintrc pathplanning/api.py pathplanning/registry.py pathplanning/search2d.py

lint-google-legacy:
	pylint --rcfile .pylintrc pathplanning

format:
	ruff format .

precommit:
	pre-commit run --all-files

typecheck:
	@command -v pyright >/dev/null || (echo "pyright not found. Run: make install-dev" && exit 1)
	pyright --project pyrightconfig.json

test:
	pytest -q

benchmark:
	python scripts/benchmark_planners.py

build:
	rm -rf dist
	python -m build

publish-test: build
	twine upload --repository testpypi dist/*

publish: build
	twine upload dist/*
