#!/bin/sh
sudo renice -n -20 $$
make -j16
make prog
