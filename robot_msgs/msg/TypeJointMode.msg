# This message holds joint operational mode
# Motor and magnetic clutch control
#
# This is a helper type and using as a part of other messages

uint8 BREAK=0   # Motor on   Clutch on
uint8 STOP=1    # Motor on   Clutch off
uint8 RELAX=2   # Motor off  Clutch off
uint8 TRACE=3   # Motor on   Clutch on   commands receiving

uint8 mode