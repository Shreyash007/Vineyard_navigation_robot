#!/bin/bash

echo "delete the remapping rule for ttyRobot"
echo "sudo rm   /etc/udev/rules.d/robot.rules"
sudo rm   /etc/udev/rules.d/robot.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
