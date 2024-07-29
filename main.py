import time
from machine import Pin
import ds18x20
import onewire

def constrain(val, min_val, max_val):
    return max(min_val, min(max_val, val))

class PIDController:
    def __init__(self, set_value=30.0, k_p=0.4, k_i=0.1, k_d=0.1, cycle=2.0, valve=60.0, dead_zone=2.0):
        # Ініціалізація змінних
        self.SET_VALUE = set_value
        self.K_P = k_p
        self.K_I = k_i
        self.K_D = k_d
        self.CYCLE = cycle
        self.VALVE = valve
        self.DEAD_ZONE = dead_zone

        # Ініціалізація змінних для ПІД-регулятора
        self.E_1 = 0.0
        self.E_2 = 0.0
        self.E_3 = 0.0
        self.SUM_D_T = 0.0
        self.TIMER_PID = 0.0
        self.TIMER_PID_UP = 0.0
        self.TIMER_PID_DOWN = 0.0
        self.PID_PULSE = 0
        self.HAND_UP = False
        self.HAND_DOWN = False
        self.ON_OFF = True
        self.AUTO_HAND = True
        self.PULSE_100MS = 0
        self.TIMER_100MS = 0.0
        self.MILLIS_FLOAT_1 = 0.0
        self.MILLIS_FLOAT_VALVE = 0.0
        self.D_T = 0.0
        self.UP = False
        self.DOWN = False

        # Налаштування GPIO
        self.RELAY_UP_PIN = Pin(3, Pin.OUT)
        self.RELAY_DOWN_PIN = Pin(4, Pin.OUT)

        # Ініціалізація датчика DS18B20
        self.ds_pin = Pin(21)
        self.ds_sensor = ds18x20.DS18X20(onewire.OneWire(self.ds_pin))
        self.roms = self.ds_sensor.scan()
        if self.roms:
            self.sensor_available = True
            self.PRESENT_VALUE = self.read_temperature()  # Ініціалізація температури
        else:
            self.sensor_available = False
            self.PRESENT_VALUE = 0.0

        # Ініціалізація часу старту
        self.start_time = time.time()

    def read_temperature(self):
        # Зчитування температури з датчика DS18B20
        try:
            if self.sensor_available:
                self.ds_sensor.convert_temp()
                time.sleep(1)  # Затримка для зчитування температури
                temp = self.ds_sensor.read_temp(self.roms[0])
                
                if temp is not None:
                    return temp
        except Exception as e:
            print(f"Error reading temperature: {e}")
        return 0.0  # Повертаємо значення за замовчуванням у разі помилки або відсутності сенсора

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
            print("Программа завершена пользователем")

    def update(self):
        MILLIS_FLOAT = float(time.ticks_ms())
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
        print(f"E_1: {self.E_1:.2f}, D_T: {self.D_T:.2f}, SUM_D_T: {self.SUM_D_T:.2f}, TIMER_PID_UP: {self.TIMER_PID_UP:.2f}, TIMER_PID_DOWN: {self.TIMER_PID_DOWN:.2f}, UP: {self.UP}, DOWN: {self.DOWN}, Elapsed Time: {elapsed_time:.2f} sec")

if __name__ == "__main__":
    controller = PIDController()
    controller.run()
