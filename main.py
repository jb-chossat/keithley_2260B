import time
from Keithley_2260B_lib import *

PS = keithley_2260B(1080, "/dev/ttyACM0")
PS.print_all_info()

PS.set_power_supply_mode("CVHS")
PS.set_voltage_current(10, 50)  # voltage / current targets
PS.set_over_current_protection(115)  # set current limit
PS.set_over_voltage_protection(32)  # set voltage limit

while 1:

    PS.power_supply_on()
    time.sleep(2)

    PS.power_supply_off()
    time.sleep(2)


# while 1:
    # print(PS.measure_voltage())
    # print(PS.measure_current())
    # print(PS.measure_power())

#TODO:
# check for relative bounds and return warnings in set methods ?
# use doc strings to detail ALL methods -> Improve / standardize documentation
# standardize all naming conventions
# add basic coil class, and measure of generated magnet field
