img=duckietown/challenge-aido1_dummy_sim-simulation:v3

all:
	echo To do from scratch:    make build-nocache push define-challenge

push:
	docker push $(img)

build:
	docker build -t $(img) . 

build-no-cache:
	docker build -t $(img) . --no-cache

define-challenge:
	dts challenges define --config ../challenge.yaml --build evaluator:.

define-challenge-no-cache:
	dts challenges define --config ../challenge.yaml --build evaluator:. --no-cache
