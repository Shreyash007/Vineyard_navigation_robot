#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  ttyRobotBase"
echo "robot usb connection as /dev/ttyRobotBase , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy robot.rules to  /etc/udev/rules.d/"
echo "`rospack find deltaxl_bringup`/scripts/robot.rules"
sudo cp `rospack find deltaxl_bringup`/scripts/robot.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
