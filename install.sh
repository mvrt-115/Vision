#!/bin/sh

#Install dependencies needed for stuff for Linux

#Run as root
if ! [ $(id -u) = 0 ];
        then echo "Please run this script as root";
        exit;
fi

#Stuff to install
pip install pupil-apriltags
pip install opencv-python
pip install pynetworktables