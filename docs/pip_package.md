# Standalone Pip Package

We provide a standalone and self-contained Pip package with full functionality and all dependencies baked into the package, so you do not have to worry about any dependencies and conflicts.

It is as simple as running:
```
pip3 install pointcloudcrafter
```

(requires pip3: `sudo apt-get install python3-pip`)

## Source Build

If you want to build the standalone pip package from source yourself, then follow these steps:

First, build the Docker environment. Follow the steps in [Docker](https://tumftm.github.io/PointCloudCrafter/docker.html).

Inside the container, execute the following commands:
```
python3 -m build --wheel --no-isolation -Cbuild-dir=/ros_ws/python/build

LD_LIBRARY_PATH=$(find /ros_ws/python/build -name "*.so*" -exec dirname {} + | sort -u | tr '\n' ':'):$LD_LIBRARY_PATH \
auditwheel repair /ros_ws/dist/*.whl -w /ros_ws/dist/auditwheel/
```
This creates the `.whl` file that is needed for the Pip installation.

To install it in the container:
```
pip3 install /ros_ws/dist/auditwheel/*.whl
```
If you want to install the self-contained standalone package locally (**without the need of any dependency locally installed!**):
```
# In container
mv /ros_ws/dist/auditwheel/*.whl /datasets/.

# In local environment
pip3 install /path/to/data/directory/*.whl
```