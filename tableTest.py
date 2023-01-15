
import sys
import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

#if len(sys.argv) != 2:
#    print("Error: specify an IP to connect to!")
#    exit(0)

#ip = sys.argv[1]

NetworkTables.initialize(server= '127.0.0.1')
#NetworkTables.initialize(server= 'roborio-20-frc.local')

sd = NetworkTables.getTable("SmartDashboard")
auto_value = sd.getAutoUpdateValue("robotTime", 0)
#print(auto_value)
poses = sd.putNumber(key='123', value=1)
#print(sd.getNumber('123', 0))
while True:
    print("robotTime:", auto_value.value)
    time.sleep(1)