#!/bin/bash
#To install vs code
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update &&
sudo apt install -y software-properties-common apt-transport-https wget &&
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - &&
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" &&
# sudo apt-get install code

pip install shapely
pip install numba
pip install open3d

# to install lsof
sudo apt-get update -y
sudo apt-get install -y lsof

pip install pynvml
pip install pyntcloud
pip install einops
pip install opencv-python
pip install dictor
pip install ephem
pip install py-trees
pip install imageio
pip install pillow
pip install tabulate
pip install ujson

pip3 install mat4py
pip install timm
pip install "laspy[laszip]"
pip install -r requirements.txt