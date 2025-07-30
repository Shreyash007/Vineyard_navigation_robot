#!/bin/bash

echo "remap the IMU serial port (ttyUSBX) to  ttyRazorIMU"
echo "imu usb connection as /dev/ttyRazorIMU , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy razorIMU.rules to  /etc/udev/rules.d/"
echo "`rospack find deltaxl_bringup`/scripts/razorIMU.rules"
sudo cp `rospack find deltaxl_bringup`/scripts/razorIMU.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
