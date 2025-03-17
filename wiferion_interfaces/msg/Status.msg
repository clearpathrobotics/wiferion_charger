# Wiferion Charger Status
float32 output_voltage
float32 output_current

uint8 IDLE=0
uint8 CHARGING=1
uint8 CHARGING_CONST_CURRENT=2
uint8 CHARGING_CONST_VOLTAGE=3
uint8 CHARGING_PRE_CHARGE=4
uint8 CHARGING_SECOND_CONST_VOLTAGE=5
uint8 BATTERY_FULL=6
uint8 STOPPED=7
uint8 ERROR=8
uint8 state
