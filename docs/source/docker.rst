Running with Docker
===================

Running with Docker can simplify continuous integration pipelines and can
also be used to just run the program without installing anything.


Building the container image
----------------------------

You can build the image without any special flags. From a shell in the root of
the repository, execute the following:

.. codeblock:: bash

    docker build -t onshape-to-robot .

Running as a container
----------------------

Make sure you've built the image as described above.

Set up environment variables with access credentials and map them through.
Mount a local directory including the `config.json` to `/workspace` inside
the container. Run as the current user to ensure correct file permissions.

For example, the following will export the current working directory:

.. codeblock:: bash

    docker run --rm -t \
        --volume="$(pwd):/workspace" \
        --user "$(id -u):$(id -g)" \
        --env=ONSHAPE_API="${ONSHAPE_API}" \
        --env=ONSHAPE_ACCESS_KEY="${ONSHAPE_ACCESS_KEY}" \
        --env=ONSHAPE_SECRET_KEY="${ONSHAPE_SECRET_KEY}" \
        onshape-to-robot
