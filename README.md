# ASPS-Power firmware

This is the ASPS-Power firmware, built using the Energia framework.

Note that this needs the ASPS-Power variant board definition located in the
'hardware' directory in this github. 

Navigate to where Energia is installed: inside the 'hardware' directory,
dig down to find the msp430/ directory. It may be in hardware/msp430 or
hardware/energia/msp430 based on the version.

Then *append* boards.txt.ara-daq-hw to boards.txt in that directory. Also copy
all directories in the 'variants' directory into the 'variants' directory
in that same location.

Now launch Energia, and select the board in Energia with
'Tools->Board->ASPS-POWER w/MSP430F2274', and upload the program.