FROM node:12-alpine
RUN apk add --no-cache python g++ make
WORKDIR /app
COPY . .
# Install python 3.7
RUN apt-get update \
    && apt-get install software-properties-common -y \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get install -y python3.7 \
        python3-distutils \
        python3-pip \
        python3-apt \
        python3.7-dev \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y python3.10 \
    && apt-get install -y python3.10-dev \
        python3.10-venv \
        python3.10-distutils \
        python3.10-lib2to3 \
        python3.10-gdbm \
        python3.10-tk \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2 \
    && update-alternatives  --set python3 /usr/bin/python3.7 \
    && update-alternatives  --set python /usr/bin/python3.7

# Make python 3.7 the default
RUN echo "alias python3=python3.7" >> ~/.bashrc \
    && echo "alias python=python3.7" >> ~/.bashrc \
    && export PATH=${PATH}:/usr/bin/python3.7 \
    && /bin/bash -c "source ~/.bashrc" \
    && apt-get remove -y python3-apt \
    && apt-get install -y python3-venv \
        python3-pip \
        python3-apt \
    && python3 -m pip install --upgrade pip

RUN yarn install --production
RUN mkdir /home/mnt_drive /home/mnt_drive1 /home/mnt_drive2
ADD get-pip.py /home/mnt_drive/get-pip.py
CMD ["node", "src/index.js"]
ENTRYPOINT ["/bin/sh", "/run.sh"]
