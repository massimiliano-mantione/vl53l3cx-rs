#!/bin/bash
#take care to unexport GPIO used on raspberry with PCB 1535 + 1536 rev B combination for L3CX satellite usage
echo 12 > /sys/class/gpio/unexport
echo 16 > /sys/class/gpio/unexport
echo 19 > /sys/class/gpio/unexport

#actual driver source code location to adapt to your case
export DRIVER_DIR=/home/pi/VerifTest/LinuxDriver/trunk/driver/vl53Lx/
export PHIO_DIR=${DRIVER_DIR}/../../android/hardware/vl53lx_test/

#Full kernel driver build
cd ${DRIVER_DIR}
make clean
make VL53LX_FULL_KERNEL=1 
sudo dtoverlay -R
sudo dtoverlay stmvl53lx
sudo insmod stmvl53lx.ko
sudo chmod 777 /dev/stmvl53lx_ranging

#phio test application build
cd ${PHIO_DIR}
make clean
make

./phio -t=100000 -s -Z=10 -S -t=16000

#here move to release testing
export unset VL53LX_FULL_KERNEL=1
cd ~/VerifTest/LinuxDriverTests/trunk/vl53lx_testes
source setDirforTest.sh
unload

