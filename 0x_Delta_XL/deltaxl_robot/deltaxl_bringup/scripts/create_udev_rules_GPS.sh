#!/bin/bash

echo "remap the GPS serial port (ttyUSBX) to  ttyGPS"
echo "GPS usb connection as /dev/ttyGPS , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy gps.rules to  /etc/udev/rules.d/"
echo "`rospack find deltaxl_bringup`/scripts/gps.rules"
sudo cp `rospack find deltaxl_bringup`/scripts/gps.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
