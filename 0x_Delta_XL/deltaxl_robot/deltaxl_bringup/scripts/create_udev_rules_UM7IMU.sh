#!/bin/bash

echo "remap the UM7 IMU serial port (ttyUSBX) to  ttyUM7IMU"
echo "imu usb connection as /dev/ttyUM7IMU , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy um7IMU.rules to  /etc/udev/rules.d/"
echo "`rospack find deltaxl_bringup`/scripts/um7IMU.rules"
sudo cp `rospack find deltaxl_bringup`/scripts/um7IMU.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
