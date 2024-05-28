import network
import time
import sensor
import socket
import struct
import ujson

UDP_RECEIVER_IP = "192.168.0.3"
UDP_RECEIVER_PORT = 57121

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
clock = time.clock()

f = open("secrets.json", "r")
secrets = ujson.loads(f.read())
f.close()
wifi_ssid = secrets["wifi"]["ssid"]
wifi_key = secrets["wifi"]["key"]

# Init wlan module and connect to network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(wifi_ssid, wifi_key)

while not wlan.isconnected():
    print('Trying to connect to "{:s}"...'.format(wifi_ssid))
    time.sleep_ms(1000)

# We should have a valid IP now via DHCP
print("WiFi Connected ", wlan.ifconfig())

# Set up a UDP socket for sending data.
udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
    # print(clock.fps())  # Note: OpenMV Cam runs about half as fast when connected
    # to the IDE. The FPS should increase once disconnected.
    histogram = img.get_histogram()
    statistics = histogram.get_statistics()
    histoMedian = statistics.median()
    histoMin = statistics.min()
    histoMax = statistics.max()
    udpSocket.sendto(struct.pack("<fff", histoMedian, histoMin, histoMax),
        (UDP_RECEIVER_IP, UDP_RECEIVER_PORT))
    time.sleep_ms(100)
