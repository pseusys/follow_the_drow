.ONESHELL:
.EXPORT_ALL_VARIABLES:
.DEFAULT_GOAL := help

SHELL = /bin/bash
PATH := venv/bin:$(PATH)



help:
	@ # Display help information
	echo ":)"
.PHONY: help



venv:
	@ # Create python virtual environment
	python3 -m venv venv
	pip3 install --upgrade pip
	pip3 install -e stdlib
	pip3 install jupyter~=1.0



redrow-detector-test: venv
	@ # Run DROW detector test
	jupyter nbconvert --execute --to notebook --inplace compare/redrow_detector.ipynb
.PHONY: redrow-detector-test



clean:
	rm -rf venv
	rm -rf compare/cache
.PHONY: clean
