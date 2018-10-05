img=duckietown/aido1_luck-v3-evaluator:latest


all: push

push: build
	docker push $(img)

build:
	docker build -t $(img) . 

build-no-cache:
	docker build -t $(img) . --no-cache
