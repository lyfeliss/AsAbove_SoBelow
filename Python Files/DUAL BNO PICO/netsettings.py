import network

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

SSID = "transfunctioner_bunker"
PASSWORD = "8479blue"

if not wlan.isconnected():
    wlan.connect(SSID, PASSWORD)

print("Connecting to Wi-Fi...", end="")
while not wlan.isconnected():
    pass

print("\nConnected! IP:", wlan.ifconfig()[0])
