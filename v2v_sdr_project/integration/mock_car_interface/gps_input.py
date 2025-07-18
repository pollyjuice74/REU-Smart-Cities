import random
from datetime import datetime

USE_MOCK = True  # Flag to switch mode

def get_mock_gps_data():
    base_lat = 52.5200
    base_lon = 13.4050
    return {
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "latitude": base_lat + random.uniform(-0.0005, 0.0005),
        "longitude": base_lon + random.uniform(-0.0005, 0.0005),
        "speed": round(random.uniform(0, 30), 2),
        "heading": round(random.uniform(0, 359), 2)
    }

def get_real_gps_data(serial_port="/dev/ttyUSB0", baudrate=9600):
    import serial
    import pynmea2
    with serial.Serial(serial_port, baudrate, timeout=1) as ser:
        while True:
            line = ser.readline().decode(errors="ignore")
            if line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                if msg.status == 'A':
                    return {
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "latitude": msg.latitude,
                        "longitude": msg.longitude,
                        "speed": round(float(msg.spd_over_grnd) * 0.51444, 2),
                        "heading": float(msg.true_course or 0.0)
                    }

def get_latest_gps(mock=USE_MOCK):
    return get_mock_gps_data() if mock else get_real_gps_data()
