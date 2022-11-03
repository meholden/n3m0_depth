"""
Example of connecting to an autopilot via serial communication using pymavlink
"""

# Import mavutil
from pymavlink import mavutil

import time

# Create the connection
# Need to provide the serial port and baudrate
#master = mavutil.mavlink_connection("/dev/ttyS0", baud=57600)
master = mavutil.mavlink_connection("/dev/ttyS0", baud=57600, source_system=1)

# Restart the ArduSub board !
##master.reboot_autopilot()

# Send a message for QGC to read out loud
#  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,"Beep Boop Hi".encode())

### Get all information !
##while True:
##    try:
##        print(master.recv_match().to_dict())
##    except:
##        pass
##    time.sleep(0.1)
##

# Wait for packets to come in and decode
# if location is updated, check for new NMEA, if new NMEA then update log file
while True:
    msg = master.recv_match()
    if not msg:
        continue
    print("msg type: \t%s\n" % msg.get_type());
    
##    if msg.get_type() == 'HEARTBEAT':
##        print("\n\n*****Got message: %s*****" % msg.get_type())
##        print("Message: %s" % msg)
##        print("\nAs dictionary: %s" % msg.to_dict())
##        # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
##        print("\nSystem status: %s" % msg.system_status)
    

    if msg.get_type() == 'GPS_RAW_INT':
        GPS = msg.to_dict();
        print("Lat is %s" % (float(GPS["lat"])/1e7))
        print("Lon is %s" % (float(GPS["lon"])/1e7))
        

print "All done"


