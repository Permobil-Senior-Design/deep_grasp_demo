#!/bin/bash

# read cmd line inputs
VERSION=$1 # cpu or gpu

# set cpu/gpu conditional libraries
case "${VERSION}"
in
cpu)
	pip install tensorflow==1.15.0
	;;
gpu)
  pip install tensorflow-gpu==1.13.1
	;;
*)
	echo "Usage: $0 {cpu|gpu}"
	exit 1
esac

# install apt deps
sudo apt install cmake libvtk6-dev python-vtk6 python-sip python-qt4 libosmesa6-dev meshlab libhdf5-dev

# install pip deps
python3 -m pip install -r dexnet_requirements.txt


