#!/bin/bash

## change these to whatever you actually need
command1="cd ~/Drone/SlapDrone/slapDrone_perch; source devel/setup.bash; bash"
command2="cd ~/Drone/SlapDrone/slapDrone_perch; source devel/setup.bash; bash"
command3="cd ~/Drone/SlapDrone/slapDrone_perch; source devel/setup.bash; bash"
command4="cd ~/Drone/SlapDrone/slapDrone_perch; source devel/setup.bash; bash"
command5="cd ~/Drone/SlapDrone/slapDrone_perch; source devel/setup.bash; bash"

## Modify terminator's config
sed -i.bak "s#PERCH1#$command1#; s#PERCH2#$command2#; s#PERCH3#$command3#; s#PERCH4#$command4#; s#PERCH5#$command5#" ~/.config/terminator/config

## Launch a terminator instance using the new layout
terminator -l perch & sleep 2;

## Return the original config file
mv ~/.config/terminator/config.bak ~/.config/terminator/config
