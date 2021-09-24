# install python
# reference to https://stackoverflow.com/questions/65644782/how-to-install-pip-for-python-3-9-on-ubuntu-20-04
apt update
apt install software-properties-common
add-apt-repository ppa:deadsnakes/ppa -y
apt update
apt install python3.9
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1
# install pip3 for python3.9
# referece to https://stackoverflow.com/questions/65644782/how-to-install-pip-for-python-3-9-on-ubuntu-20-04
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
apt install python3-distutils
python3.9 get-pip.py

# uninstall python
apt-get remove --auto-remove python3.x
apt-get purge --auto-remove python3

# install from source
https://linoxide.com/install-python-3-9-on-ubuntu-20-04-lts/
