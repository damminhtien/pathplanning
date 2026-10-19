NATIVE_BUILD_DIR ?= /tmp/pathplanning-native-build
LOCAL_BIN ?= $(CURDIR)/.venv/bin

.PHONY: install install-dev lint lint-google lint-native lint-coverage format precommit test test-unit test-slow test-all typecheck benchmark build-ext

install:
	pip install .

install-dev:
	pip install -r requirements-dev.txt
	npm ci
	$(MAKE) build-ext

lint:
	PATH="$(LOCAL_BIN):$$PATH" pre-commit run --all-files

lint-google:
	pylint --rcfile .pylintrc pathplanning

format:
	PATH="$(LOCAL_BIN):$$PATH" ruff format .
	npm run format:web

precommit:
	PATH="$(LOCAL_BIN):$$PATH" pre-commit run --all-files

typecheck:
	python3 scripts/run_pyright.py

lint-native:
	python3 scripts/lint_native.py

lint-coverage:
	python3 scripts/check_lint_coverage.py

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
