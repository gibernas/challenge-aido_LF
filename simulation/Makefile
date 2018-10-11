

tag=local-simulation


build:
	docker build -t $(tag) .

build-base-no-cache:
	docker build -t $(tag) --no-cache .

develop:
	docker run -it -v $(PWD):/pwd -w /pwd $(tag) bash
