repo=challenge-aido_lf-duckiebot
branch=$(shell git rev-parse --abbrev-ref HEAD)
tag=duckietown/$(repo):$(branch)
arch=amd64

build:
	docker build --pull -t $(tag) --build-arg ARCH=$(arch) .

build-no-cache:
	docker build --pull -t $(tag) --build-arg ARCH=$(arch) --no-cache .

push: build
	docker push $(tag)
