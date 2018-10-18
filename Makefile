define-challenge:
	$(MAKE) define-challenge-LF
	$(MAKE) define-challenge-LFV

define-challenge-no-cache:
	$(MAKE) define-challenge-LF-no-cache
	$(MAKE) define-challenge-LFV-no-cache

define-challenge-LF:
	dts challenges define --config LF.challenge.yaml

define-challenge-LFV:
	dts challenges define --config LFV.challenge.yaml

define-challenge-LF-no-cache:
	dts challenges define --config LF.challenge.yaml  --no-cache

define-challenge-LFV-no-cache:
	dts challenges define --config LFV.challenge.yaml  --no-cache
