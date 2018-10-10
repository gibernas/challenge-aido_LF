img=duckietown/gym-duckietown-server-ros:v3

all:


push-base:
	docker push $(img)

build-base:
	docker build -t $(img) -f Dockerfile.gym_duckietown_server_ros .

build-base-no-cache:
	docker build -t $(img)-f Dockerfile.gym_duckietown_server_ros  . --no-cache



build:
	docker build .

build-base-no-cache:
	docker build --no-cache .

