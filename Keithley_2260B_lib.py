import serial
import logging
from threading import Thread, RLock
import time
import numpy as np


class keithley_2260B:
    """
    Overview:
    The Keithley 2260B 30-108 is high power (1080 W) high max current power supply that is controllable using a USB to
    serial connection. This python class implements the power supply main functions, but many other functions are
    available were you to search in the programming manual.

    This power supply has programmable current and voltage slew rates, variable internal resistance, and multiple power
    supplies can be connected in series (30 and 80 V models only) and in parallel.

    Theory of Operation:
    In constant current mode, the power supply will output a variable voltage and constant current, unless either
    the total power used exceeds the PS maximum (1080 W) or if the load resistance increases so that the
    set current (Iset) cannot be maintained.
    In constant voltage mode, the power supply will output a varRiable current and constant voltage, unless the load
    resistance is too low to maintain a constant voltage, in which case it will then operate in constant current mode
    and maintain the current limit.

    The conditions that determine whether the power supply operates in CC or CV mode depends on the set current (Iset),
    the set voltage (Vset), the load resistance (RL) and the critical resistance (RC). The critical resistance is
    determined by Vset/Iset. The power supply will operate in CV mode when the load resistance is greater than the
    critical resistance. This means that the voltage output will be equal to the Vset voltage but the current will be
    less than Iset. If the load resistance is reduced to the point that the current output reaches the
    Iset level, the power supply switches to CC mode. Conversely the power supply will operate in CC mode when the load
    resistance is less than the critical resistance. In CC mode the current output is equal to Iset and the voltage
    output is less than Vset.
    """

    ps_info_string = []
    ps_model_string = []

    max_voltage = 0.0
    min_voltage = 0.0
    max_current = 0.0
    min_current = 0.0
    max_power = 0.0

    over_current_protection_min = 0.0
    over_current_protection_max = 0.0
    over_voltage_protection_min = 0.0
    over_voltage_protection_max = 0.0

    current_max_rising_slew_rate = 0.0
    current_min_rising_slew_rate = 0.0
    current_max_falling_slew_rate = 0.0
    current_min_falling_slew_rate = 0.0

    voltage_max_rising_slew_rate = 0.0
    voltage_min_rising_slew_rate = 0.0
    voltage_max_falling_slew_rate = 0.0
    voltage_min_falling_slew_rate = 0.0

    internal_resistance_min = 0.0
    internal_resistance_max = 0.0

    def __init__(self, max_power, serial_path):
        """Creates the power supply object and initialize connection using serial protocol.

        The power supply max and min boundary values for voltage, current, internal resistance, voltage slew rates,
        current slew rates, over-voltage protection and over-current protection are automatically extracted and
        populate local variables when connecting to the power supply.
        This allows to check for boundary values when sending instructions to the power supply.

        Only the PS max power must be specified at object creation. Max voltage, current and power can be found on the
        PS front panel for check.

        :param power: Max power supply power
        :param serial_path: the usb uart path on your computer. For ex. "/dev/ttyACM0" on a proper operating system
        (linux).
        """
        self.dev = serial.Serial(serial_path, 115200)
        self.max_power = max_power

        self.get_power_supply_info()
        print("Successfully connected to: " + self.ps_info_string)

        # self.ps_model_string = self.ps_info_string.split(',')[1]
        # self.max_voltage = float(self.ps_model_string.split('-')[1])
        # self.max_current = float(self.ps_model_string.split('-')[2])

        self.max_current = self.get_current("MAX")
        self.min_current = self.get_current("MIN")
        self.max_voltage = self.get_voltage("MAX")
        self.min_voltage = self.get_voltage("MIN")

        self.current_max_rising_slew_rate = self.get_rising_current_slew_rate("MAX")
        self.current_min_rising_slew_rate = self.get_rising_current_slew_rate("MIN")
        self.current_max_falling_slew_rate = self.get_falling_current_slew_rate("MAX")
        self.current_min_falling_slew_rate = self.get_falling_current_slew_rate("MIN")

        self.voltage_max_rising_slew_rate = self.get_rising_voltage_slew_rate("MAX")
        self.voltage_min_rising_slew_rate = self.get_rising_voltage_slew_rate("MIN")
        self.voltage_max_falling_slew_rate = self.get_falling_voltage_slew_rate("MAX")
        self.voltage_min_falling_slew_rate = self.get_falling_voltage_slew_rate("MIN")

        self.internal_resistance_min = self.get_internal_resistance("MIN")
        self.internal_resistance_max = self.get_internal_resistance("MAX")

        self.over_current_protection_min = self.get_over_current_protection("MIN")
        self.over_current_protection_max = self.get_over_current_protection("MAX")
        self.over_voltage_protection_min = self.get_over_voltage_protection("MIN")
        self.over_voltage_protection_max = self.get_over_voltage_protection("MAX")

        # Multithreading & reading data related

        # To prevent reading buffers while it's getting filled
        self.lock = RLock()
        #  Flag used for quitting the thread
        self.reading_thread_active = False
        #  Create a thread to read the scope's data
        self.ps_thread = Thread(target=self._read_power_supply_data)

        # Data structures
        self.data_current = []
        self.data_voltage = []
        self.data_power = []


        print("Power supply object initialized.\n")

    def print_all_info(self):
        """ Prints all info collected on the power supply in a human readable format.
        Useful to check the power supply absolute minimum and maximum capabilities.
        :return:
        """
        print("Power supply: " + self.ps_info_string)
        print("Created by the user as a " + str(self.max_power) + "W power supply.")
        print("Power supply internal resistance: MIN=" + str(self.internal_resistance_min) + "Ohm / MAX="
              + str(self.internal_resistance_max) + "Ohm / SET= " + str(self.get_internal_resistance()) + "Ohm")
        print("The power supply is in " + self.get_power_supply_mode() + " mode.\n")

        print("Current: MIN=" + str(self.min_current) + "A / MAX=" + str(self.max_current) + "A / SET="
              + str(self.get_current()) + "A")
        print("Current rising slew rates: MIN=" + str(self.current_min_rising_slew_rate) + "A/s / MAX=" +
              str(self.current_max_rising_slew_rate) + "A/s / SET=" +
              str(self.get_rising_current_slew_rate()) + "A/s")
        print("Current falling slew rates: MIN=" + str(self.current_min_falling_slew_rate) + "A/s / MAX=" +
              str(self.current_max_falling_slew_rate) + "A/s / SET=" +
              str(self.get_falling_current_slew_rate()) + "A/s")
        print("Over-current protection: MIN=" + str(self.over_current_protection_min) + "A / MAX=" +
              str(self.over_current_protection_max) + "A / SET=" + str(self.get_over_current_protection()) + "A\n")

        print("Voltage: MIN=" + str(self.min_voltage) + "V / MAX=" + str(self.max_voltage) + "V / SET=" +
              str(self.get_voltage()) + "V")
        print("Voltage rising slew rates: MIN=" + str(self.voltage_min_rising_slew_rate) + "V/s / MAX=" +
              str(self.voltage_max_rising_slew_rate) + "V/s / SET=" + str(self.get_rising_voltage_slew_rate()) + "V/s")
        print("Voltage falling slew rates: MIN=" + str(self.voltage_min_falling_slew_rate) + "V/s / MAX=" +
              str(self.voltage_max_falling_slew_rate) + "V/s / SET=" + str(self.get_falling_voltage_slew_rate()) + "V/s")
        print("Over voltage protection: MIN=" + str(self.over_voltage_protection_min) + "V/s / MAX=" +
              str(self.over_voltage_protection_max) + "V/s / SET=" + str(self.get_over_voltage_protection()) + "V/s")

    def write(self, string):
        """ Writes the command to the power supply using the Serial protocol.
        Will properly encode the string and add termination characters.

        :param string: The character string to send to the power supply.
        :return:
        """
        self.dev.write((string + "\n").encode("utf-8"))

    def read(self):
        """
        :return: A string coming from the serial port.
        It is decoded and stripped from termination characters.
        """
        return self.dev.readline().decode('utf-8').strip()

    def abort(self):
        """ Used to "cancel any triggered action". Not sure when to use.
        :return:
        """
        self.write("ABOR")

    def set_voltage_current(self, voltage, current):
        """ Sets new voltage and current targets, Will check for out of bounds conditions.
        Still requires the PS "power_supply_on" function.
        These are not reset when the power supply is powered off.

        :param voltage: float
        :param current: float
        :return:
        """
        if self.min_voltage < voltage < self.max_voltage and self.min_current < current < self.max_current:
            self.write("APPL " + str(voltage) + "," + str(current))
        else:
            logging.warning("Warning: Set voltage or current out of power supply absolute ratings. "
                            "Command has been ignored.")

    def get_voltage_current(self):
        """ Get the PS voltage and current settings.
        :return: A tuple of floats that represents first the PS's voltage, and second the PS's current
        """
        self.write("APPL?")
        string = self.read().split(',')
        return float(string[0]), float(string[1])

    def measure_current(self):
        """ Measure the PS current.
        :return: A float representing the current in amps.
        """
        self.write("MEAS:CURR?")
        return float(self.read())

    def measure_voltage(self):
        """ Measures the PS voltage.
        :return: A float representing the voltage in volts.
        """
        self.write("MEAS:VOLT?")
        return float(self.read())

    def measure_power(self):
        """ Measures the PS power in Watts. Is seems to be rounded up to a single digit.
        :return: A float representing the power in watts.
        """
        self.write("MEAS:POW?")
        return float(self.read())

    def set_internal_resistance(self, resistance):
        """Sets the power supply internal resistance. Will be checked for our of bounds conditions.
        This is not reset when the power supply is powered off.
        :param resistance: The PS internal resistance in ohms.
        :return:
        """
        if self.internal_resistance_min < resistance < self.internal_resistance_max:
            self.write("RES " + str(resistance))
        else: logging.warning("Warning: Set internal resistance is out of power supply absolute ratings. "
                              "Command has been ignored.")

    def get_internal_resistance(self, min_max=None):
        """ Gets the PS internal resistance in ohms.
        :param min_max: Optional parameter to get the PS min and max possible internal resistance.
        :return: A float such 0.286 or 0.0
        """
        if min_max is None:
            self.write("RES?")
        elif min_max == "MIN":
            self.write("RES? MIN")
        elif min_max == "MAX":
            self.write("RES? MAX")
        return float(self.read())

    def set_power_supply_mode(self, ps_mode):
        """ Sets the power supply mode between Constant Voltage High Speed,
        Constant Current High Speed, Constant Voltage Low Speed, and Constant Current Low Speed.
        High Speed modes use the fastest slew rates available (216 A/s or 60V/s for the 30-108 model for ex.),
        while Low Speed modes use the slew rates set by the user.
        This is not reset when the power supply is powered off.
        Will check for malformed string command.

        :param ps_mode: "CVHS", "CCHS", "CVLS", or "CCLS".
        :return:
        """

        if ps_mode in (("CVHS"), ("CCHS"), ("CVLS"), ("CCLS")): self.write("OUTP:MODE " + ps_mode)
        else: logging.warning("Warning: Malformed string for power supply mode command. "
                              "Command has been ignored.")

    def get_power_supply_mode(self):
        """ Indicates the PS operation mode: Constant Voltage High Speed,
        Constant Current High Speed, Constant Voltage Low Speed, or Constant Current Low Speed.
        High Speed modes use the fastest slew rates available (216 A/s or 60V/s), while Low Speed modes use the
        slew rates set by the user.
        :return: String of characters indicating the PS mode. CVHS, CCHS, CVLS or CCLS.
        """
        str_mode = ""
        self.write("OUTP:MODE?")
        ps_mode = self.read()
        if ps_mode == '0': str_mode = "CVHS"
        elif ps_mode == '1': str_mode = "CCHS"
        elif ps_mode == '2': str_mode = "CVLS"
        elif ps_mode == '3': str_mode = "CCLS"

        return str_mode

    def power_supply_on(self):
        """ The power supply output is on, voltage and current are generated according to the chosen settings.
        :return:
        """
        self.write("OUTP 1")

    def power_supply_off(self):
        """ The power supply output is off and no voltage and currents are actively generated.
        :return:
        """
        self.write("OUTP 0")

    def set_current(self, current_value):
        """ Sets the power supply output target current in amps. Will be checked with PS absolute ratings.
        This is not reset when the power supply is powered off.
        :param current_value: A float, will be casted as string.
        :return:
        """
        if self.min_current <  current_value < self.max_current:
            self.write("CURR "+ str(current_value))
        else: logging.warning("Warning: Set current target is out of power supply absolute ratings. "
                              "Command has been ignored.")

    def get_current(self, min_max=None):
        """ Returns the PS targeted current output. Use of "MIN" and "MAX" strings allows to get the PS respective
        minimum and maximum current capabilities. WARNING. THE MAX BOUNDARY IS ABOVE THE POWER SUPPLY CONSTANT RATING
        AND SHOULD BE TREATED WITH CAUTION.
        :param min_max: Optional: "MIN" or "MAX" character string.
        :return: A float representing the PS current in amps.
        """
        if min_max is None:
            self.write("CURR?")
        elif min_max == "MAX":
            self.write("CURR? MAX")
        elif min_max == "MIN":
            self.write("CURR? MIN")

        return float(self.read())

    def set_over_current_protection(self, over_current_protection):
        """ Sets the PS over-current protection. This is not reset when the power supply is powered off.
        Will check for out of bounds conditions.
        :param over_current_protection: A float, represents the PS limit in amps.
        :return:
        """
        if self.over_current_protection_min < over_current_protection < self.over_current_protection_max:
            self.write("CURR:PROT " + str(over_current_protection))

        else: logging.warning("Warning: Set over-current protection is out of power supply absolute ratings. "
                              "Command has been ignored.")

    def get_over_current_protection(self, min_max=None):
        if min_max is None:
            self.write("CURR:PROT?")
        elif min_max == "MIN":
            self.write("CURR:PROT? MIN")
        elif min_max == "MAX":
            self.write("CURR:PROT? MAX")

        return float(self.read())
        # self.over_current_protection = float(self.read())
        # return self.over_current_protection

    def set_rising_current_slew_rate(self, slew_rate):
        """ Sets the PS rising current slew rate. Will check for out of bounds conditions.
        This is not reset when the power supply is powered off.
        :param slew_rate: The PS slew rate in A/s.
        :return:
        """
        if self.current_min_rising_slew_rate < slew_rate < self.current_max_falling_slew_rate:
            self.write("CURR:SLEW:RIS " + str(slew_rate))
        else: logging.warning("Warning: Set rising current slew rate is out of power supply absolute ratings. "
                              "Command has been ignored.")

    def get_rising_current_slew_rate(self, min_max=None):
        """
        :param min_max: Optional character string "MIN" or "MAX" to get the PS min and max respective rising current
        slew rates.
        :return: A float that indicates the min, max or actual PS current rising slew rate.
        """
        if min_max is None:
            self.write("CURR:SLEW:RIS?")
        elif min_max == "MAX":
            self.write("CURR:SLEW:RIS? MAX")
        elif min_max == "MIN":
            self.write("CURR:SLEW:RIS? MIN")

        return float(self.read())

    def set_over_voltage_protection(self, over_voltage_protection):
        """ Sets the PS over-voltage protection. Will check for out of bounds conditions.
        This is not reset when the power supply is powered off.
        :param over_voltage_protection:
        :return:
        """
        if self.over_current_protection_min < over_voltage_protection < self.over_voltage_protection_max:
            self.write("VOLT:PROT " + str(over_voltage_protection))
        else: logging.warning("Warning: Set over-voltage protection is out of power supply absolute ratings. "
                              "Command has been ignored.")

    def get_over_voltage_protection(self, min_max=None):
        if min_max is None:
            self.write("VOLT:PROT?")
        elif min_max == "MIN":
            self.write("VOLT:PROT? MIN")
        elif min_max == "MAX":
            self.write("VOLT:PROT? MAX")

        return float(self.read())

    def set_falling_current_slew_rate(self, slew_rate):
        """ Sets the PS falling current slew rate. Will check for out of bounds conditions.
        This is not reset when the power supply is powered off.
        :param slew_rate:
        :return:
        """
        if self.current_min_falling_slew_rate < slew_rate < self.current_max_falling_slew_rate:
            self.write("CURR:SLEW:FALL " + str(slew_rate))
        else: logging.warning("Warning: Set falling current slew rate is out of power supply absolute ratings. "
                              "Command has been ignored.")

    def get_falling_current_slew_rate(self, min_max=None):
        if min_max is None:
            self.write("CURR:SLEW:FALL?")
        elif min_max == "MIN":
            self.write(("CURR:SLEW:FALL? MIN"))
        elif min_max == "MAX":
            self.write(("CURR:SLEW:FALL? MAX"))

        return float(self.read())

    def set_voltage(self, voltage_value):
        """ Sets the power supply voltage output target.  Will check for out of bounds conditions.
        This is not reset when the power supply is powered off.
        :param voltage_value: A float representing the
        :return:
        """
        if self.min_voltage < voltage_value < self.max_voltage:
            self.write("VOLT "+ str(voltage_value))
        else: logging.warning("Warning: Set voltage target is out of power supply absolute ratings. "
                              "Command has been ignored.")

    def get_voltage(self, min_max=None):
        """ Returns the PS targeted voltage output. Use of "MIN" and "MAX" strings allows to get the PS respective
        minimum and maximum voltage capabilities. WARNING. THE MAX BOUNDARY IS ABOVE THE POWER SUPPLY CONSTANT RATING
        AND SHOULD BE TREATED WITH CAUTION.
        :param min_max: Optional: "MIN" or "MAX" character string.
        :return: A float representing the PS voltage in volts.
        """
        if min_max is None:
            self.write("VOLT?")
        elif min_max == "MAX":
            self.write("VOLT? MAX")
        elif min_max == "MIN":
            self.write("VOLT? MIN")

        return float(self.read())

    def set_rising_voltage_slew_rate(self, slew_rate):
        """ Sets the PS rising voltage slew rate. Will check for out of bounds conditions.
        This is not reset when the power supply is powered off.
        :param slew_rate:
        :return:
        """
        if self.voltage_min_rising_slew_rate < slew_rate < self.voltage_max_rising_slew_rate:
            self.write("VOLT:SLEW:RIS " + str(slew_rate))
        else:
            logging.warning("Warning: Set rising voltage slew rate is out of power supply absolute ratings. "
                            "Command has been ignored.")

    def get_rising_voltage_slew_rate(self, min_max=None):
        """Returns the PS voltage rising slew rate.
        :param min_max:  Optional character string "MIN" or "MAX" to get the PS min and max rising voltage slew rate.
        :return: A float
        """
        if min_max is None:
            self.write("VOLT:SLEW:RIS?")
        elif min_max == "MIN":
            self.write("VOLT:SLEW:RIS? MIN")
        elif min_max == "MAX":
            self.write("VOLT:SLEW:RIS? MAX")
        return float(self.read())

    def set_falling_voltage_slew_rate(self, slew_rate):
        """ Sets the PS falling voltage slew rate.  Will check for out of bounds conditions.
        This is not reset when the power supply is powered off.
        :param slew_rate:
        :return:
        """
        if self.voltage_min_falling_slew_rate < slew_rate < self.voltage_max_falling_slew_rate:
            self.write("VOLT:SLEW:FALL " + str(slew_rate))
        else:
            logging.warning("Warning: Set falling voltage slew rate is out of power supply absolute ratings. "
                            "Command has been ignored.")

    def get_falling_voltage_slew_rate(self, min_max=None):
        """ Returns the PS voltage falling slew rate.
        :param min_max: Optional character string "MIN" or "MAX" to get the PS min and max falling voltage slew rate.
        :return: A float
        """
        if min_max is None:
            self.write("VOLT:SLEW:FALL?")
        elif min_max == "MIN":
            self.write("VOLT:SLEW:FALL? MIN")
        elif min_max == "MAX":
            self.write("VOLT:SLEW:FALL? MAX")

        return float(self.read())

    def get_power_supply_info(self):
        self.write("SYST:INF?")
        self.ps_info_string = self.read()

    def start_thread_power_supply_data(self):
        """
        Initiate a thread that will continuously read the power supply's voltage, current and power.
        """
        self.reading_thread_active = True
        self.ps_thread = Thread(target=self._read_power_supply_data)
        self.ps_thread.start()

    def stop_thread_power_supply_data(self):
        """
        Stops reading voltage, current and power from the power supply.
        """
        self.reading_thread_active = False
        time.sleep(0.1)
        self.ps_thread.join()

    def _read_power_supply_data(self):
        """
        Multithreaded safe method that continuously reads and acquires the power supply's voltage, current and power.
        """
        while self.reading_thread_active:
            self.lock.acquire()  # Acquire lock for multithreading purposes

            self.data_voltage.append(self.measure_voltage())
            self.data_current.append(self.measure_current())
            self.data_power.append(self.measure_power())

            self.lock.release()  # Release lock for multithreading purposes

            #  Delay so that other processes can execute as well
            time.sleep(0.05)

    def save_data(self):
        """
        Saves the power supply current, voltage and power data in CSV file named "PS_data.csv"
        """
        ps_data = [[], [], []]
        ps_data[0] = np.array(self.data_voltage)
        ps_data[1] = np.array(self.data_current)
        ps_data[2] = np.array(self.data_power)

        with open('PS_data.csv', 'w') as f:
            np.savetxt(f, ps_data, delimiter=',')

    def start_power_supply(self):
        """
        Starts both the power supply output as defined before, as well as the power supply data capture (voltage,
        current and power)
        """
        self.power_supply_on()
        self.start_thread_power_supply_data()

    def stop_power_supply(self):
        """
        Stops the power supply's output as well as data reading & saving.
        """
        self.power_supply_off()
        time.sleep(0.2)
        self.stop_thread_power_supply_data()
        self.save_data()
