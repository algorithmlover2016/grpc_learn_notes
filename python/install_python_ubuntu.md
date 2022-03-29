# [install python](https://stackoverflow.com/questions/65644782/how-to-install-pip-for-python-3-9-on-ubuntu-20-04)
```sh
apt update
apt install software-properties-common
add-apt-repository ppa:deadsnakes/ppa -y
apt update
apt install python3.9
# cd /usr/bin; rm -rf python3 && ln -s python3.9 python3
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1
update-alternatives --install /usr/bin/python3 python3 /root/anaconda3/envs/planercnn1/bin/python3.7 2
# reference to https://blog.csdn.net/l1216766050/article/details/82796409
# update-alternatives --config python
```

# [install pip3 for python3.9](https://stackoverflow.com/questions/65644782/how-to-install-pip-for-python-3-9-on-ubuntu-20-04)
```sh
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
apt install python3-distutils
python3.9 get-pip.py
```
or<br>
```sh
# Installing pip3 in Ubuntu(https://www.educative.io/edpresso/installing-pip3-in-ubuntu)
sudo apt-get update
sudo apt-get -y install python3-pip
pip3 --version
```

# uninstall python
```
apt-get remove --auto-remove python3.x
apt-get purge --auto-remove python3
```

# [install from source](https://linoxide.com/install-python-3-9-on-ubuntu-20-04-lts/)
