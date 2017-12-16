#!/bin/sh
wget http://kevinboone.net/r2d2-voice.tar.gz
tar xvf r2d2-voice.tar.gz
cp r2d2-voice/src/main.c ../src/r2d2-voice.c
rm -rf r2d2-voice*
