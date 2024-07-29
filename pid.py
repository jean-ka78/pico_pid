import time
import config
import wifi_config
from machine import Pin
import onewire
import ds18x20

class PIDController:
    def __init__(self):
        # Ініціалізація конфігурації
        self.SET_VALUE = config.SET_VALUE
        self.K_P = config.K_P
        self.K_I = config.K_I
        self.K_D = config.K_D
        self.CYCLE = config.CYCLE
        self.VALVE = config.VALVE
        self.DEAD_ZONE = config.DEAD_ZONE

        # Ініціалізація пінів
        self.RELAY_UP_PIN = Pin(config.RELAY_UP_PIN, Pin.OUT)
        self.RELAY_DOWN_PIN = Pin(config.RELAY_DOWN_PIN, Pin.OUT)

        # Ініціалізація Wi-Fi
        self.wifi = wifi_config.connect_wifi(config.SSID, config.PASSWORD)

        # Ініціалізація сенсора
        self.ds_sensor = onewire.OneWire(Pin(config.SENSOR_PIN))
        self.roms = self.ds_sensor.scan()
        self.sensor_available = len(self.roms) > 0
        self.PRESENT_VALUE = 0.0

        self.start_time = time.time()

    def read_temperature(self):
        try:
            if self.sensor_available:
                self.ds_sensor.convert_temp()
                time.sleep(1)
                temp = self.ds_sensor.read_temp(self.roms[0])
                if temp is not None:
                    return temp
        except Exception as e:
            print(f"Error reading temperature: {e}")
        return 0.0

    def run(self):
        try:
            while True:
                if self.sensor_available:
                    self.PRESENT_VALUE = self.read_temperature()
                self.update()
                self.control()
                self.print_status()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def update(self):
        MILLIS_FLOAT = float(time.time() * 1000)
        MILLIS_FLOAT_SEK = MILLIS_FLOAT / 1000.0

        self.E_1 = self.SET_VALUE - self.PRESENT_VALUE

        if self.K_I == 0.0:
            self.K_I = 9999.0

        self.K_P = constrain(self.K_P, -99.0, 99.0)
        self.K_I = constrain(self.K_I, 1.0, 9999.0)
        self.K_D = constrain(self.K_D, 0.0, 9999.0)
        self.CYCLE = constrain(self.CYCLE, 1.0, 25.0)
        self.VALVE = constrain(self.VALVE, 15.0, 250.0)
        self.DEAD_ZONE = constrain(self.DEAD_ZONE, 0.0, 9999.0)

        if self.TIMER_PID == 0.0 and not self.PID_PULSE:
            self.PID_PULSE = 1
            self.MILLIS_FLOAT_1 = MILLIS_FLOAT_SEK
            self.D_T = self.K_P * (self.E_1 - self.E_2 + self.CYCLE * self.E_2 / self.K_I +
                                   self.K_D * (self.E_1 - 2 * self.E_2 + self.E_3) / self.CYCLE) * self.VALVE / 100.0
            self.E_3 = self.E_2
            self.E_2 = self.E_1
            self.SUM_D_T += self.D_T
            if self.SUM_D_T >= 0.5:
                self.TIMER_PID_DOWN = 0.0
            if self.SUM_D_T <= -0.5:
                self.TIMER_PID_UP = 0.0
            if -self.DEAD_ZONE < self.E_1 < self.DEAD_ZONE:
                self.D_T = 0.0
                self.SUM_D_T = 0.0

        self.TIMER_PID = MILLIS_FLOAT_SEK - self.MILLIS_FLOAT_1
        if self.ON_OFF and self.AUTO_HAND:
            if self.TIMER_PID >= self.CYCLE:
                self.PID_PULSE = 0
                self.TIMER_PID = 0.0
                if self.SUM_D_T >= 0.5 or self.SUM_D_T <= -0.5:
                    self.SUM_D_T = 0.0
            if self.TIMER_PID < 0.0:
                self.MILLIS_FLOAT_1 = MILLIS_FLOAT_SEK
        else:
            self.PID_PULSE = 0
            self.D_T = 0.0
            self.SUM_D_T = 0.0
            self.TIMER_PID = 0.0
            self.E_3 = self.E_1
            self.E_2 = self.E_1
            self.TIMER_PID_UP = 0.0
            self.TIMER_PID_DOWN = 0.0

        if self.TIMER_100MS == 0.0 and not self.PULSE_100MS:
            self.PULSE_100MS = 1
            self.MILLIS_FLOAT_VALVE = MILLIS_FLOAT_SEK
        else:
            self.PULSE_100MS = 0

        self.TIMER_100MS = MILLIS_FLOAT_SEK - self.MILLIS_FLOAT_VALVE
        if self.TIMER_100MS >= 0.1 or self.TIMER_100MS < 0.0:
            self.TIMER_100MS = 0.0

    def control(self):
        if not self.sensor_available:
            self.RELAY_UP_PIN.value(0)
            self.RELAY_DOWN_PIN.value(0)
            return

        self.UP = (((self.SUM_D_T >= self.TIMER_PID and self.SUM_D_T >= 0.5) or self.D_T >= self.CYCLE - 0.5 or self.TIMER_PID_UP >= self.VALVE) and self.AUTO_HAND or (self.HAND_UP and not self.AUTO_HAND)) and self.ON_OFF and not self.DOWN
        if self.UP and self.TIMER_PID_UP < self.VALVE:
            self.TIMER_PID_UP += 0.1
            self.RELAY_UP_PIN.value(1)
        else:
            self.RELAY_UP_PIN.value(0)

        self.DOWN = (((self.SUM_D_T <= -self.TIMER_PID and self.SUM_D_T <= -0.5) or self.D_T <= -self.CYCLE + 0.5 or self.TIMER_PID_DOWN >= self.VALVE) and self.AUTO_HAND or (self.HAND_DOWN and not self.AUTO_HAND)) and self.ON_OFF and not self.UP
        if self.DOWN and self.TIMER_PID_DOWN < self.VALVE:
            self.TIMER_PID_DOWN += 0.1
            self.RELAY_DOWN_PIN.value(1)
        else:
            self.RELAY_DOWN_PIN.value(0)

    def print_status(self):
        elapsed_time = time.time() - self.start_time
        print(f"E_1: {self.E_1:.2f}, D_T: {self.D_T:.2f}, SUM_D_T: {self.SUM_D_T:.2f}, "
              f"TIMER_PID_UP: {self.TIMER_PID_UP:.2f}, TIMER_PID_DOWN: {self.TIMER_PID_DOWN:.2f}, "
              f"UP: {self.UP}, DOWN: {self.DOWN}, Elapsed Time: {elapsed_time:.2f} sec")
