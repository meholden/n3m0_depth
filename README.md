# n3m0_depth
companion computer code for n3m0 with depth sensor, data pushed to a different repo on github

This project uses pymavlink on a raspberry pi connected to an ardupilot flight controller's serial port

Also connected to the pi is a NMEA 0183 depth sensor

Data from sensor is logged with telemetry data into a geojson file.  

Tide offset is calculated and logged too (from internet lookup)

Geojson file is committed to github every so often (if internet is available)
