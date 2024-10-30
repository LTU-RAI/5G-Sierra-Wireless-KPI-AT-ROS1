#!/bin/bash

#load driver
sudo modprobe option

#inject driver option in kernel to the device ID (Sierra EM9293)
sudo sh -c "echo '1199 90e3' > /sys/bus/usb-serial/drivers/option1/new_id"


