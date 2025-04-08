# Wiferion Charger State
uint32 serial_number
uint32 version_major
uint32 version_minor
uint32 version_revision
# Heatsink temperature
float32 heatsink_temperature
# Mobile coil temperature
float32 coil_temperature
# Temperature at HF1 terminal of ME
float32 hf1_temperature
# Temperature at HF2 terminal of ME
float32 hf2_temperature
# Temperature at Battery (+) terminal
float32 positive_temperature
# Temperature at Battery (-) terminal
float32 negative_temperature
# Reference charge current
float32 ref_charge_current
# Reference charge voltage
float32 ref_charge_voltage
# BMS Type
uint8 NONE=0
uint8 GENERIC=1
uint8 SCIB=2
uint8 ETASTORE=4
uint8 UNKNOWN=0xFF
uint8 bms_type
