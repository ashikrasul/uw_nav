import serial 
import time 
import adafruit_gps
uart=serial.Serial('/dev/ttyUSB0',baudrate=9600,timeout=10)
gps=adafruit_gps.GPS(uart,debug=False)

gps.send_command(b'PMTK251,115200')
uart.baudrate = 115200

uart.flush
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,100')

last_print = time.monotonic()

while True:
    check_gps=gps.update()

    if check_gps==True:
        print(gps.nmea_sentence)

    current=time.monotonic()
    if current - last_print >= 1.0: 
        last_print=current
        if not gps.has_fix:
            print("GPS No Fix...")
            continue
        print('=' * 40)  # Print a separator line.
        print('Latitude: {0:.6f} degrees'.format(gps.latitude))
        print('Longitude: {0:.6f} degrees'.format(gps.longitude))
        print(f'Fix Quality:{gps.fix_quality}')
        print(f'Satellites:{gps.satellites}')
        print(f'Altitude: {gps.altitude_m} meters')
        print(f'GeoID Seperation (N): {gps.height_geoid} meters')
        print(f'Horizontal Dilution of Precision: {gps.horizontal_dilution}')
        print(f'Timestamp (UTC): {gps.timestamp_utc}')
