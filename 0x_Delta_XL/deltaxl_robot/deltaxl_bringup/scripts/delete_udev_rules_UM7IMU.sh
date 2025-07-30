#!/bin/bash

echo "delete the remapping rule for ttyUM7IMU"
echo "sudo rm   /etc/udev/rules.d/um7IMU.rules"
sudo rm   /etc/udev/rules.d/um7IMU.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
