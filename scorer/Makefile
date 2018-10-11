img=duckietown/challenge-aido1_dummy_sim-scoring:v3


all: push

push: build
	docker push $(img)

build:
	docker build -t $(img) .

build-no-cache:
	docker build -t $(img) . --no-cache

develop:
	docker run -it -v $(PWD):/pwd -w /pwd $(img) bash
