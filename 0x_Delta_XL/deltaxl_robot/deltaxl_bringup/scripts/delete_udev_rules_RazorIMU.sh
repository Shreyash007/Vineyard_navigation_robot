#!/bin/bash

echo "delete the remapping rule for ttyRazorIMU"
echo "sudo rm   /etc/udev/rules.d/razorIMU.rules"
sudo rm   /etc/udev/rules.d/razorIMU.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
