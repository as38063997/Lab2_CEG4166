import RPi.GPIO as GPIO
import time
import threading

# (Optional) Turn off GPIO warnings if you get "already in use" warnings
GPIO.setwarnings(False)

# Set pin numbering mode
GPIO.setmode(GPIO.BOARD)

class HCSR04:
    """
    Simple HC-SR04 ultrasonic sensor driver.
    """
    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo

        # Setup pins
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

        # Initialize trigger pin to Low
        GPIO.output(self.trig, False)
        # Give the sensor a moment to settle
        time.sleep(0.2)

    def measure(self, samples=5, unit="cm"):
        """
        Perform multiple measurements and return the averaged distance.
        
        :param samples: number of samples to average
        :param unit: 'cm' for centimeters or 'inch' for inches
        :return: averaged distance in the specified unit or None on timeout
        """
        accumulator = 0.0

        for _ in range(samples):
            # Send 10us pulse to trigger
            GPIO.output(self.trig, True)
            time.sleep(0.00001)
            GPIO.output(self.trig, False)

            start = time.time()
            pulse_start = 0
            pulse_end = 0

            # Wait for echo to go HIGH
            while GPIO.input(self.echo) == 0:
                if time.time() - start > 0.05:  # 50ms timeout
                    return None
                pulse_start = time.time()

            # Wait for echo to go LOW
            while GPIO.input(self.echo) == 1:
                if time.time() - start > 0.05:  # 50ms timeout
                    return None
                pulse_end = time.time()

            # Calculate pulse duration
            pulse_duration = pulse_end - pulse_start
            # Distance in centimeters (speed of sound ~34300 cm/s, round trip factor of 2)
            distance_cm = pulse_duration * 17150

            if unit.lower() == "inch":
                # Convert cm -> inches
                distance = distance_cm / 2.54
            else:
                # Default to cm
                distance = distance_cm

            accumulator += distance

        average_distance = accumulator / samples
        return round(average_distance, 2)

def sonar_loop(sensor, samples, stop_event):
    """
    Continuously measure distance and print results until stop_event is set.
    """
    try:
        while not stop_event.is_set():
            distance = sensor.measure(samples, "cm")
            if distance is not None:
                print("Distance:", distance, "cm")
                if distance < 10:
                    print("Object detected! Stopping/rerouting...")
            else:
                print("Sensor timeout or error.")
            
            # Adjust the sleep time as needed for your application
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Sonar thread interrupted by user.")
    finally:
        print("Sonar loop finishing.")

def main():
    # Create sensor instance on pins 7 (TRIG) and 12 (ECHO)
    sensor = HCSR04(trig=7, echo=12)

    # Event to signal the sonar thread to stop
    stop_event = threading.Event()

    # Create and start the sonar thread
    samples = 5
    sensor_thread = threading.Thread(target=sonar_loop, args=(sensor, samples, stop_event))
    sensor_thread.start()

    try:
        # Main thread doing other stuff
        # For demonstration, just wait here until user hits Ctrl+C
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Main program interrupted by user.")
    finally:
        # Signal the sonar thread to stop
        stop_event.set()
        sensor_thread.join()
        # Cleanup GPIO once, at the very end
        GPIO.cleanup()
        print("GPIO cleanup done. Program exiting.")

if __name__ == "__main__":
    main()
