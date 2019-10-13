repo=challenge-aido_lf-duckiebot
branch=$(shell git rev-parse --abbrev-ref HEAD)
tag=duckietown/$(repo):$(branch)

build:
	docker build --pull -t $(tag) .

build-no-cache:
	docker build --pull -t $(tag) --no-cache .

push: build
	docker push $(tag)
