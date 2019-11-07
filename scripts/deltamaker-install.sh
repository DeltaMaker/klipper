#!/bin/bash
# This script copies the deltamaker printer config and octoprint config files to
#  the required locations
#
printer_cfg="/home/pi/printer.cfg"
config_yaml="/home/pi/.octoprint/config.yaml"

deltamaker_config="/home/pi/klipper/config/printer-deltamaker-r2-touch.cfg"
octoprint_config="/home/pi/klipper/config/octoprint/config-deltamaker.yaml"
if [ -f "$printer_cfg" ]; then
    echo "Exiting: $printer_cfg already exists"
    exit -1
fi
#rm -f $printer_cfg 
if [ -f "$deltamaker_config" ]; then
    echo "Installing $deltamaker_config as $printer_cfg"
    cp $deltamaker_config $printer_cfg
else
    echo "$deltamaker_config not found"
fi

if [ -f "$octoprint_config" ]; then
    echo "Installing $octoprint_config as $config_yaml"
    cp $octoprint_config $config_yaml
else
    echo "$octoprint_config not found"
fi