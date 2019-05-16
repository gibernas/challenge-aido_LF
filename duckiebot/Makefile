repo=challenge-aido_lf-duckiebot
branch=$(shell git rev-parse --abbrev-ref HEAD)
tag=duckietown/$(repo):$(branch)

build:
	docker build -t $(tag) .

build-no-cache:
	docker build -t $(tag) --no-cache .

push: build
	docker push $(tag)
