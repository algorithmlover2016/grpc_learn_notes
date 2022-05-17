# [run command]<br>
* `docker run --ipc=host -it --cpus=4 --memory="8g" --memory-swap="-1" --name "docker_name" image_name:tag  /bin/bash`<br>
* `docker run -it --gpus all --name PlaneRcnnTest -e LANG=C.UTF-8 --entrypoint /bin/bash -v /data:/data nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04`<br>

# [docker without sudo](https://docs.docker.com/engine/install/linux-postinstall/)
```sh
sudo groupadd docker
sudo usermod -aG $USER
newgrp docker
```

