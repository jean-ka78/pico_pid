import network
import time

SSID = 'aonline'
PASSWORD = '1qaz2wsx3edc'

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if wlan.isconnected():
        print('Already connected to Wi-Fi')
        return
    
    print(f'Connecting to {ssid}...')
    wlan.connect(ssid, password)
    
    # Wait until the connection is established
    timeout = 10  # seconds
    start_time = time.time()
    
    while not wlan.isconnected() and time.time() - start_time < timeout:
        time.sleep(1)
    
    if wlan.isconnected():
        print('Connected to Wi-Fi')
        print(f'IP Address: {wlan.ifconfig()[0]}')
    else:
        print('Failed to connect to Wi-Fi')
        return None
    
    return wlan
