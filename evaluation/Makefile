img=duckietown/aido1_dummy_sim:v3


all: push

push: build
	docker push $(img)

build:
	docker build -t $(img) . 

build-no-cache:
	docker build -t $(img) . --no-cache
