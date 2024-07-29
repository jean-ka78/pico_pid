from pid import PIDController
import config
import wifi_config

def main():
    # Підключення до Wi-Fi
    wifi_config.connect_wifi(wifi_config.SSID, wifi_config.PASSWORD)

    # Ініціалізація PID контролера
    pid_controller = PIDController(config)

    # Запуск PID контролера
    pid_controller.run()

if __name__ == "__main__":
    main()
