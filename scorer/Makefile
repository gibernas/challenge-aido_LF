img=duckietown/aido1_amod1-aidoscorer:latest


all: push

push: build
	docker push $(img)

build:
	docker build -t $(img) .

build-no-cache:
	docker build -t $(img) . --no-cache

