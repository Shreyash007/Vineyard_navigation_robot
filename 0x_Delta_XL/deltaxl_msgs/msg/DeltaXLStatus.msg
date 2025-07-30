Header header

# MCU Uptime, in ms
uint32 uptime

# ROS Control loop frequency (PC-side)
float64 ros_control_loop_freq

# Current draw of platform components, in amps
float64 total_current
float64 total_motor_driver_current
float64 left_motor_current
float64 right_motor_current
float64 system_current
float64 payload_current
float64 auxillary_power_current

# Voltage of platform components, in volts
float64 battery_voltage
float64 motor_voltage
float64 payload_supply_voltage
float64 aux_supply_voltage

# Component temperatures, in C
float64 motor_driver_temp
float64 left_motor_temp
float64 right_motor_temp
float64 payload_bay_temp
float64 battery_bay_temp
float64 battery_sensor1_temp
float64 battery_sensor2_temp
float64 fuse_box_temp

# Battery capacity (Wh) and charge (%) estimate
uint16 capacity_estimate
float64 charge_estimate

# Delta XL error/stop conditions
bool timeout
bool lockout
bool e_stop
bool current_limit
bool battery_present
