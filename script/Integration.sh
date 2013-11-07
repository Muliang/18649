#!/bin/bash

set -e
set -u

rm -f *.stats

java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario1A.cf -mf integration_test/Scenario1A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario1B.cf -mf integration_test/Scenario1B.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario1C.cf -mf integration_test/Scenario1C.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario2A.cf -mf integration_test/Scenario2A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario2B.cf -mf integration_test/Scenario2B.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario2C.cf -mf integration_test/Scenario2C.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario3A.cf -mf integration_test/Scenario3A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario4A.cf -mf integration_test/Scenario4A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario5A.cf -mf integration_test/Scenario5A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario5B.cf -mf integration_test/Scenario5B.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario6A.cf -mf integration_test/Scenario6A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario7A.cf -mf integration_test/Scenario7A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario7B.cf -mf integration_test/Scenario7B.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario7C.cf -mf integration_test/Scenario7C.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario7D.cf -mf integration_test/Scenario7D.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario8A.cf -mf integration_test/Scenario8A.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario8B.cf -mf integration_test/Scenario8B.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario8C.cf -mf integration_test/Scenario8C.mf | grep "Message"
java simulator/framework/Elevator -rt 0s -head HEADER  -cf integration_test/Scenario9A.cf -mf integration_test/Scenario9A.mf | grep "Message"


grep "Failed" *.stats

rm -f *.stats
