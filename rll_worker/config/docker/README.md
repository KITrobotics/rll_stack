# RLL Docker containers

You can find the Docker container configs in this folder.

Some needed projects are checked out via submodules:

     `git submodule update --init --recursive`

Then you can enter one of the folders and build the respective container, e.g.:

     `docker build --network host -t rll-base .`

This is all you need for processing jobs for a specific project using the worker.

If you want to manually run an image for testing , you can do so with:

   `docker run --net=host -it rll-base`
