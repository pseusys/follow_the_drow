.ONESHELL:
.EXPORT_ALL_VARIABLES:
.DEFAULT_GOAL := help

SHELL = /bin/bash
PATH := venv/bin:$(PATH)

ROBAIR_IP=192.168.1.201
CONTAINER_IP := $(shell hostname -I | awk '{print $$1}')


help:
	@ # Display help information
	echo ":)"
.PHONY: help



venv:
	@ # Create python virtual environment
	python3 -m venv venv
	pip3 install --upgrade pip jupyter~=1.0
	pip3 install -e library
	jupyter nbextension enable --py widgetsnbextension

build-lib:
	@ # Build and install `follow_the_drow` library locally
	mkdir -p library/cpp_core/build
	cd library/cpp_core/build
	cmake ..
	make
	test $$(id -u) -eq 0 && { make install; } || { echo "Installation can be performed in superuser mode only!"; }
.PHONY: build-lib



redrow-detector-test: venv
	@ # Run DROW detector test
	jupyter nbconvert --execute --to notebook --inplace compare/redrow_detector.ipynb
.PHONY: redrow-detector-test



build-image:
	@ # Build docker image - internet connection required!
	docker compose -f deploy/docker/docker-compose.yml build
	test -n "$$(docker images -q follow_the_drow 2> /dev/null)" || { echo "Image building error!"; exit 1; }
.PHONY: build-image

launch-docker-local:
	export ROBAIR_IP=127.0.0.1
	docker compose -f deploy/docker/docker-compose.yml up --force-recreate roslaunch
.PHONY: launch-docker-local

launch-docker-robot:
	ping -q -W 1 -c 1 $(ROBAIR_IP) 2>&1 > /dev/null || { echo "Not connected to RobAIR network!"; exit 1; }
	docker compose -f deploy/docker/docker-compose.yml up --force-recreate roslaunch
.PHONY: launch-docker-robot



clean:
	rm -rf venv
	rm -rf compare/cache
	rm -rf library/cpp_core/build
	rm -rf library/follow_the_drow.egg-info
	rm -rf library/follow_the_drow/*.so
	rm -rf library/follow_the_drow/include
	rm -rf deploy/out
	rm -rf **/__pycache__
.PHONY: clean

# TODO: add different clean targets
