#!/bin/bash

set -e
set -u

rm -f *.stats

java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/HallButtonControl.cf -mf unit_test/HallButtonControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/CarButtonControl.cf -mf unit_test/CarButtonControl.mf| grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/drivecontrol.cf -mf unit_test/drivecontrol.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/CarPositionControl.cf -mf unit_test/CarPositionControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/LanternControl.cf -mf unit_test/LanternControl.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER -cf unit_test/Dispatcher.cf -mf unit_test/Dispatcher.mf | grep "Message"

grep "Failed" *.stats

rm -f *.stats
