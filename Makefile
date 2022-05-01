IMAGE ?= onshape_to_robot

all:
	@rm -rf dist/*
	python3 setup.py sdist bdist_wheel

docker-build:
	docker build . -t $(IMAGE)

upload:
	python3 -m twine upload --repository pypi dist/*

upload-test:
	python3 -m twine upload --repository testpypi dist/*

clean:
	rm -rf build dist onshape_to_robot.egg-info
