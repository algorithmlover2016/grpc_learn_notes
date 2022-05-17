# [run command]<br>
* `docker run --ipc=host -it --cpus=4 --memory="8g" --memory-swap="-1" --name "docker_name" image_name:tag  /bin/bash`<br>
* `docker run -it --gpus all --name PlaneRcnnTest -e LANG=C.UTF-8 --entrypoint /bin/bash -v /data:/data nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04`<br>

# [docker without sudo](https://docs.docker.com/engine/install/linux-postinstall/)
```sh
sudo groupadd docker
sudo usermod -aG $USER
newgrp docker
```

# [add user in docker container]
```sh
# Taken from - https://docs.docker.com/engine/examples/running_ssh_service/#environment-variables
RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

# 22 for ssh server. 7777 for gdb server.
EXPOSE 22 7777

RUN useradd -ms /bin/bash ${your-user-name}
RUN echo '${your-user-name}:${your-passwd}' | chpasswd

WORKDIR /home/${your-user-name}
RUN chmod -R a+w /home/${your-user-name}
```