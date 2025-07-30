#!/bin/bash

echo "delete the remapping rule for ttyGPS"
echo "sudo rm   /etc/udev/rules.d/gps.rules"
sudo rm   /etc/udev/rules.d/gps.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
