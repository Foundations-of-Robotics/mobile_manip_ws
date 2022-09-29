# This message represents Dingo's lower-frequency status updates
# Default publish frequency is 1Hz.

Header header

# Commit of firmware source.
string hardware_id
string firmware_version

# Times since MCU power-on and MCU rosserial connection, respectively.
duration mcu_uptime
duration connection_uptime

# Temperature of MCU's PCB in Celsius.
float32 pcb_temperature
float32 mcu_temperature

# Monitoring the run/stop loop. Changes in these values trigger an immediate
# publish, outside the ordinarily-scheduled 1Hz updates.
#bool stop_power_status      # True indicates the stop loop is operational (ie. it is powered).
bool stop_engaged           # True when a stop has been pressed on the robot.
bool drivers_active         # False when a stop is asserted to the motor drivers.
bool external_stop_present  # True indicates a external stop has been plugged in.

# Power connected readings
bool shore_power_connected  # Indicates if AC power is connected.
bool battery_connected      # Indicates if battery is connected.
bool power_12v_user_nominal # Indicates if the 12V user power is good.

# Voltage rails, in volts
# Averaged over the message period
float32 measured_battery
float32 measured_12v
float32 measured_5v

# Current senses available on platform, in amps.
# Averaged over the message period
float32 computer_current
float32 total_current
