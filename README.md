# OLD KS6e_ACU firmware, no longer used and was the new code that Matthew had started to write
KS6e ACU firmware
Testing branch

3 states:
0 = setting channels and moving to state 1
1 = waiting to check readings, if past 100ms, set to state 2 to get readings
2 = getTemps and set state back to 0, and cycle through channels 0-11

Functionality:
-Recieve HI/LOW temp data from BMS
-Send HI/LOW temp data from MBD to BMS, only reporting the highest/lowest values
-cycle between channels to switch between modules
-switch between states
-get temps from MDBs
-monitor IMD PWM 
-print debugging
-control fans
