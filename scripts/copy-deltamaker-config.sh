#!/bin/bash
# This script copies the deltamaker printer config and octoprint config files to
#  the required locations

if [ -e ~/printer.cfg ]; then
    echo "printer.cfg already exists"
    exit -1
fi
#rm -f ~/printer.cfg 
config_file="~/klipper/config/printer-deltamaker-r2-touch.cfg"
echo "Installing $config_file in home directory"
cp $config_file ~/

octoprint_config="~/klipper/config/octoprint/config-deltamaker.yaml"
echo $octoprint_config
if [ -e $octoprint_config ]; then
    echo "Installing $octoprint_config as ~/.octoprint/config.yaml"
    cp $octoprint_config ~/.octoprint/config.yaml
fi
