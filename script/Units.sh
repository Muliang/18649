#!/bin/bash

set -e
set -u

rm -f *.stats

java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/HallButtonControl.cf -mf unit_test/HallButtonControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/CarButtonControl.cf -mf unit_test/CarButtonControl.mf| grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/DriveControl.cf -mf unit_test/DriveControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/DoorControl.cf -mf unit_test/DoorControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/CarPositionControl.cf -mf unit_test/CarPositionControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/LanternControl.cf -mf unit_test/LanternControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/Dispatcher.cf -mf unit_test/Dispatcher.mf | grep "Message"

grep "Failed" *.stats

#rm -f *.stats
