docker run --ipc=host -it --cpus=4 --memory="8g" --memory-swap="-1" --name "docker_name" image_name:tag  /bin/bash

# run a docker container with /bin/bash command
docker run -it --gpus all --name PlaneRcnnTest -e LANG=C.UTF-8 --entrypoint /bin/bash -v /data:/data nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04