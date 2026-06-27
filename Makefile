NATIVE_BUILD_DIR ?= /tmp/pathplanning-native-build

.PHONY: install install-dev lint format precommit test test-unit test-slow test-all typecheck benchmark build-ext

install:
	pip install .

install-dev:
	pip install -r requirements-dev.txt
	$(MAKE) build-ext

lint:
	ruff check .
	ruff format .

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
	pyright

build-ext:
	python setup.py build_ext --inplace --build-temp "$(NATIVE_BUILD_DIR)/temp" --build-lib "$(NATIVE_BUILD_DIR)/lib"

test: test-unit

test-unit: build-ext
	pytest -q -m "not slow"

test-slow: build-ext
	pytest -q -m "slow"

test-all: build-ext
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
