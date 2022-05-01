FROM ubuntu:22.04

RUN mkdir /app
WORKDIR /app

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y meshlab openscad python-is-python3 python3-pip python3-setuptools python3-wheel \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
RUN python setup.py build && python setup.py install && python setup.py clean --all

WORKDIR /
RUN rm -rf /app
RUN mkdir -m777 /.cache
ENTRYPOINT ["onshape-to-robot", "/workspace"]
CMD []
