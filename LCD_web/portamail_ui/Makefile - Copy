PYTHON ?= ./venv/bin/python

.PHONY: test smoke check

test:
	$(PYTHON) -m unittest discover -s tests -t . -v

smoke:
	$(PYTHON) tools/smoke_regression.py

check: test smoke
