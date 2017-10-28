######################
#  PIN CONFIGURATION #
######################

# Setting the Bus for Pins
# by Dwindra, Dec 2016
# The pin sorted the same as pins position on board


#echo cape-bone-iio > /sys/devices/bone_capemgr.*/slots
# --> no need to echo this since the kernel version is different


#####################
#  Pin for Sensors  #
#####################
# CS
# P9_13 0_31=31
echo 31 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio31/direction

# DIN
# P9_11 0_30=30
echo 30 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio30/direction

# CLK
# P9_12 1_28=60
echo 60 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio60/direction

# === AD Board #1 ===
# DOUT1
# P9_14 1_18=50
echo 50 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio50/direction

# === AD Board #2 ===
# DOUT2
# P9_15 1_16=48
echo 48 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio48/direction

# === AD Board #3 ===
# DOUT3
# P9_26 0_14=14
echo 14 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio48/direction

# === AD Board #4 ===
# DOUT4
# P9_27 3_19=115
echo 115 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio48/direction



####################
#  Pin for Valves  #
####################

# SCLK
# P9_21 0_3=3
echo 3 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio3/direction

# MOSI
# P9_30 3_16=112
echo 112 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio112/direction

# === DAC Board #1 ===
# CS1
# P9_16 1_19=51
echo 51 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio51/direction
# CS2
# P9_42 0_7=7
echo 7 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio7/direction

# === DAC Board #2 ===
# CS3
# P9_23 1_17=49
echo 49 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio51/direction
# CS4
# P9_24 0_15=15
echo 15 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio7/direction



# Other
# P9_22 0_2=2
echo 2 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio2/direction


