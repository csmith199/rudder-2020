# rudder-2020
### Dennis Evangelista, 2019
Rudder node for AY20

## Push Test 1
For Push Test 1 (PT1), this node is not needed.

## Push Test 2
For Push Test 2 (PT2) and later, this node will listen for rudder(mainsail) commands on the NMEA 2000 bus and drive the stepper motor appropriately. It should also periodically communicate back status information for where the rudder is. Rudder and mainsail nodes are to be identical, with different instance numbers for one versus the other. 

## Notes
This node must respond to rudder PGN 127245 and should also return status periodically.

## Applicability
USNA Sailbot AY20 Hull 14 mod 3 and later.

