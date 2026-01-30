#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

GITNAME="ghcr.io/tumftm"
NAME="pointcloudcrafter"
TAG="latest"

usage() {
    echo "Usage: $0 [--name <image_name>] [--tag <image_tag>] [--help]" 1>&2;
    echo "    --gitname <image_git_name>    Set the gitname of the Docker image (default: ghcr.io/tumftm)"
    echo "    --name <image_name>           Set the name of the Docker image (default: pointcloudcrafter)"
    echo "    --tag <image_tag>             Set the tag of the Docker image (default: latest)"
    echo "    --help                        Display this help message"
    exit 1;
}


while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --gitname)
            GITNAME="$2"
            shift # past argument
            shift # past value
            ;;
        --name)
            NAME="$2"
            shift # past argument
            shift # past value
            ;;
        --tag)
            TAG="$2"
            shift # past argument
            shift # past value
            ;;
        --help)
            usage
            ;;
        *)  # unknown option
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done


docker build -f $SCRIPT_DIR/Dockerfile -t $GITNAME/$NAME:$TAG $SCRIPT_DIR/..