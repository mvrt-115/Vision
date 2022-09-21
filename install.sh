#!/bin/sh

#Install dependencies for Linux

#Run as root
if ! [ $(id -u) = 0 ];
        then echo "Please run this script as root";
        exit;
fi

pip install pupil-apriltags
pip install opencv-python
