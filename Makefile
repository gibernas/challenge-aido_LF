define-gb-challenges:
	$(MAKE) define-gb-challenge-eV
	$(MAKE) define-gb-challenge-eD
	$(MAKE) define-gb-challenge-eI
	$(MAKE) define-gb-challenge-eDI

define-gb-challenges-no-cache:
	$(MAKE) define-gb-challenge-eV-no-cache
	$(MAKE) define-gb-challenge-eD-no-cache
	$(MAKE) define-gb-challenge-eI-no-cache
	$(MAKE) define-gb-challenge-eDI-no-cache


define-gb-challenge-eV:
	dts challenges define --config LF_eV.challenge.yaml

define-gb-challenge-eD:
	dts challenges define --config LF_eD.challenge.yaml

define-gb-challenge-eI:
	dts challenges define --config LF_eI.challenge.yaml

define-gb-challenge-eDI:
	dts challenges define --config LF_eDI.challenge.yaml


define-gb-challenge-eV-no-cache:
	dts challenges define --config LF_eV.challenge.yaml --no-cache

define-gb-challenge-eD-no-cache:
	dts challenges define --config LF_eD.challenge.yaml --no-cache

define-gb-challenge-eI-no-cache:
	dts challenges define --config LF_eI.challenge.yaml --no-cache

define-gb-challenge-eDI-no-cache:
	dts challenges define --config LF_eDI.challenge.yaml --no-cache


test-with-local-repos:
	docker-compose -f docker-compose-devel.yaml down -v
	#docker-compose -f docker-compose-devel.yaml build
	docker-compose -f docker-compose-devel.yaml up -V --build

test-regular:
	docker-compose -f docker-compose.yaml down -v
	docker-compose -f docker-compose.yaml build --pull
	docker-compose -f docker-compose.yaml up -V --build

test-regular-no-pull:
	docker-compose -f docker-compose.yaml down -v
	docker-compose -f docker-compose.yaml build
	docker-compose -f docker-compose.yaml up -V --build


test-regular-no-cache:
	docker-compose -f docker-compose.yaml down -v
	docker-compose -f docker-compose.yaml build --pull --no-cache
	docker-compose -f docker-compose.yaml up -V --build
