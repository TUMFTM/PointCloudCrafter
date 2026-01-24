# Docker

## Github Container Registry

Pull the latest version of the provided docker image from the github registry:

```bash
docker pull ghcr.io/tumftm/pointcloudcrafter:latest
```

## Source Build

If you want to build the docker image yourself, run the following script:

```bash
./docker/build_docker.sh
```

To run the docker container use:

```bash
./docker/run_docker.sh /path/to/data/directory
```