# Wiferion Charger Error Code

# Charging disabled due to overtemperature
bool over_temperature
# Invalid communication with SE
bool comm_timeout
bool comm_crc_error
# Battery temperature out of limits
bool batt_temp_limit
# Time exceeded during the optional pre-charge phase
bool pre_charge_time_limit
# Voltage and/or temperature errors reported by the BMS
bool volt_temp_error
# Required CAN msg. not received in less than 3x msg. periods
bool can_message
# SE grid error
bool grid_error
# Coil not connected to SE
bool se_coil_disconnected
# SE overtemperature
bool se_over_temperature
# No charging due to bad coil positioning (too far)
bool bad_coil_position
# Fan RPM < 3000 RPM
bool fan_rpm_low
# Delivered current limited due to temperature derating
bool delivered_current_limit
# Charge current limited due to temperature derating
bool charge_current_limit
# Battery current is limited due to poor positioning (too close)
bool batt_current_limit
# Charging disabled via CAN
bool charging_disabled
# Power derating due to grid voltage
bool power_derating
# Derating due to max. power
bool max_power_derating
# SE temperature derating
bool temperature_derating
