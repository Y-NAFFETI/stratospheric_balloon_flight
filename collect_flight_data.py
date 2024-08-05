import os
import serial
import time
import adafruit_max31865
import busio
import digitalio
import board
import adafruit_dht
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
import datetime
import sys
import csv
import subprocess

# Initialize DHT22
dht_device = adafruit_dht.DHT22(board.D26)  # check GPIO connection board.GPIO<number>

def convert_to_decimal_degrees(value, direction):
    degrees = int(value / 100)
    minutes = value - (degrees * 100)
    decimal_degrees = degrees + (minutes / 60)
    if direction == 'S' or direction == 'W':
        decimal_degrees = -decimal_degrees
    return decimal_degrees

def parse_GPGGA(sentence):
    parts = sentence.split(',')
    if parts[0] == '$GPGGA':
        try:
            lat = float(parts[2]) if parts[2] else None
            lat_dir = parts[3]
            lon = float(parts[4]) if parts[4] else None
            lon_dir = parts[5]
            alt = float(parts[9]) if parts[9] else None

            if lat and lat_dir and lon and lon_dir and alt:
                lat = convert_to_decimal_degrees(lat, lat_dir)
                lon = convert_to_decimal_degrees(lon, lon_dir)
                return lat, lon, alt
        except ValueError:
            pass
    return None, None, None

def PT1000_read_temperature(sensor):
    temp = round(sensor.temperature, 2)
    return temp

def collect_DHT22_data():
    global dht_device
    try:
        temperature = dht_device.temperature
        humidity = dht_device.humidity
        return temperature, humidity
    except Exception as error:
        print(f"Error reading DHT22: {error}. Reinitializing sensor.")
        dht_device.exit()
        dht_device = adafruit_dht.DHT22(board.D26)  # check GPIO connection board.GPIO<number>
        try:
            temperature = dht_device.temperature
            humidity = dht_device.humidity
            return temperature, humidity
        except Exception as error2:
            print(f"Error reading DHT22 after reinitialization: {error2}. Skipping this reading.")
            return None, None

def get_cpu_temperature():
    try:
        result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
        temp_str = result.stdout
        temp = float(temp_str.split('=')[1].split('\'')[0])
        return round(temp, 2)
    except Exception as e:
        print(f"Error reading CPU temperature: {e}")
        return None

def get_free_space_gb():
    st = os.statvfs('/')
    free_space = st.f_bavail * st.f_frsize
    return free_space / (1024 * 1024 * 1024)  # Convert bytes to GB

def capture_image(picam2, folder, timestamp):
    filename = os.path.join(folder, f'image_{timestamp}.jpg')
    picam2.capture_file(filename)
    return filename

def start_recording_video(camera, encoder, video_path):
    output = FileOutput(video_path)
    encoder.output = output
    camera.start_encoder(encoder)

def stop_recording_video(camera, encoder):
    camera.stop_encoder(encoder)

# Create a new directory for this run
start_time = datetime.datetime.now()
run_folder = start_time.strftime("run_%d_%m_%Y_%H_%M")
os.makedirs(run_folder, exist_ok=True)

# Create the log file with the current date and time
log_filename = os.path.join(run_folder, start_time.strftime("sensor_data_log_%d_%m_%Y_%H_%M.csv"))

# Initialize log file
with open(log_filename, 'w', newline='') as log_file:
    writer = csv.writer(log_file)
    writer.writerow(['Date Time', 'Elapsed Time', 'PT1000 external Temperature (°C)', 'DHT22 internal Temperature (°C)', 'DHT22 internal Humidity (%)', 'CPU Temperature (°C)', 'Latitude (°)', 'Longitude (°)', 'Altitude (m)', 'Image Path'])

try:
    # Initialize GPS
    ser = serial.Serial('/dev/ttyAMA5', 9600, timeout=1)
    spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
    cs = digitalio.DigitalInOut(board.D5)

    # Initialize PT100
    R_REF = 4300.0
    RTD_NOMINAL = 1000.0
    PT1000 = adafruit_max31865.MAX31865(spi, cs, rtd_nominal=RTD_NOMINAL, ref_resistor=R_REF)

    # Initialize camera
    camera = Picamera2()
    video_config = camera.create_video_configuration(main={"size": (1920, 1080), "format": "RGB888"}, controls={"FrameRate": 50, "NoiseReductionMode": 1})
    config = camera.create_still_configuration(main={"size": (2048, 1536)})
    camera.configure(config)
    camera.configure(video_config)
    camera.start()
    
    # Initialize encoder
    encoder = H264Encoder(framerate=50, bitrate=10000000, enable_sps_framerate=True)
    
    # Start recording video
    video_path = os.path.join(run_folder, f"video_{datetime.datetime.now().strftime('%Y-%m-%d %H_%M_%S')}.h264")
    start_recording_video(camera, encoder, video_path)
    recording_video = True
    
    last_capture_time = time.time()
    timestamp = 0
    image_path = capture_image(camera, run_folder, timestamp)
    
    with open(log_filename, 'a', newline='') as log_file:
        writer = csv.writer(log_file)
        writer.writerow([start_time.strftime("%Y-%m-%d %H:%M:%S"), "0:00:00", "None", "None", "None", "None", "None", "None", "None", image_path])

    max_runtime = 4 * 60 * 60  # Maximum runtime of 4 hours in seconds
    end_time = start_time.timestamp() + max_runtime

    temp_above_threshold_start_time = None

    while True:
        try:
            image_taken = False
            current_time = time.time()

            if current_time >= end_time:
                print("Maximum runtime reached. Exiting...")
                sys.exit(0)  # Exit with code 0

            elapsed_time = current_time - start_time.timestamp()  # Calculate elapsed time since start
            elapsed_time_formatted = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
            date_time_now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Only read the DHT22 temperature
            DHT22_internal_temperature, DHT22_internal_humidity = collect_DHT22_data()

            if DHT22_internal_temperature is not None and DHT22_internal_temperature > 49:
                if temp_above_threshold_start_time is None:
                    temp_above_threshold_start_time = current_time
                elif current_time - temp_above_threshold_start_time > 15 * 60:  # 15 minutes in seconds
                    print("Temperature above threshold for more than 15 minutes. Exiting...")
                    sys.exit(0)  # Exit with code 0
                else:
                    print(f"Temperature above 49°C. Current temperature: {DHT22_internal_temperature:.2f}°C. Pausing data collection.")
                    time.sleep(5)
                    continue  # Skip the rest of the loop to minimize instructions
            else:
                temp_above_threshold_start_time = None

            PT1000_external_temperature = PT1000_read_temperature(PT1000)
            cpu_temperature = get_cpu_temperature()

            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$GPGGA'):
                lat, lon, alt = parse_GPGGA(line)
                if lat is not None and lon is not None and alt is not None:
                    print(f"Latitude: {lat:.6f}, Longitude: {lon:.6f}, Altitude: {alt:.2f} m")
                else:
                    print("GPS not detecting")
            else:
                lat = lon = alt = None

            free_space = get_free_space_gb()
            print(f"Free space: {free_space:.2f} GB")
            
            if (free_space < 1) and (recording_video):
                stop_recording_video(camera, encoder)
                print("Stopped recording video due to low storage")
                recording_video = False

            if not recording_video and ((current_time - last_capture_time) >= 5):
                timestamp += 1
                image_path = capture_image(camera, run_folder, timestamp)
                last_capture_time = current_time
                image_taken = True
                print(f"Image captured with timestamp {timestamp}")

            print(f"Elapsed time: {elapsed_time_formatted}")
            print(f"PT1000 external temp: {PT1000_external_temperature:.2f}°C")
            if DHT22_internal_temperature is not None and DHT22_internal_humidity is not None:
                print(f"DHT22 internal temp: {DHT22_internal_temperature:.2f}°C")
                print(f"DHT22 internal humidity: {DHT22_internal_humidity:.2f}%")
            if cpu_temperature is not None:
                print(f"CPU temp: {cpu_temperature:.2f}°C")

            with open(log_filename, 'a', newline='') as log_file:
                writer = csv.writer(log_file)
                writer.writerow([date_time_now, elapsed_time_formatted,
                                 f"{PT1000_external_temperature:.2f}" if PT1000_external_temperature is not None else "None",
                                 f"{DHT22_internal_temperature:.2f}" if DHT22_internal_temperature is not None else "None",
                                 f"{DHT22_internal_humidity:.2f}" if DHT22_internal_humidity is not None else "None",
                                 f"{cpu_temperature:.2f}" if cpu_temperature is not None else "None",
                                 lat if lat is not None else "None",
                                 lon if lon is not None else "None",
                                 alt if alt is not None else "None",
                                 image_path if image_taken else "None"])

            time.sleep(5)

        except KeyboardInterrupt:
            print("Keyboard interrupt received. Exiting...")
            sys.exit(0)  # Use exit code 0 to indicate a manual stop

        except Exception as e:
            print(f"An error occurred: {e}")
            sys.exit(1)  # Use exit code 1 to indicate an error

finally:
    try:
        dht_device.exit()
    except Exception as e:
        print(f"Error exiting DHT device: {e}")

    try:
        camera.stop()
    except Exception as e:
        print(f"Error stopping camera: {e}")

    try:
        ser.close()
    except Exception as e:
        print(f"Error closing serial port: {e}")

    print("Cleanup done")
