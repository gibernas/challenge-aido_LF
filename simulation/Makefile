img=duckietown/gym-duckietown-server-ros:v3

all:


push-base:
	docker push $(img)

build-base:
	docker build -t $(img) -f Dockerfile.gym_duckietown_server_ros .

build-base-no-cache:
	docker build -t $(img)-f Dockerfile.gym_duckietown_server_ros  . --no-cache


.PHONY: build


build:
	docker build -t local-simulator  .

build-base-no-cache:
	docker build -t local-simulator --no-cache .


test1: build
	docker run -it local-simulator python /project/test1.py

shell:
	docker run -it local-simulator bash
