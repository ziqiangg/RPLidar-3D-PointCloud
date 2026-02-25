"""
FS90 Servo Control using gpiozero
Works perfectly on Raspberry Pi 5 - No daemon required!

Wiring for Raspberry Pi 5:
- Red (5V)     → Pin 2 (5V)
- Brown (GND)  → Pin 6 (Ground)
- Orange (PWM) → Pin 12 (GPIO 18)
"""

from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory
from time import sleep

class ServoFS90:
    def __init__(self, pin):
        """
        Initialize the FS90 servo with gpiozero
        
        Args:
            pin: GPIO pin number (BCM numbering)
        """
        # Use lgpio factory for Raspberry Pi 5 support
        factory = LGPIOFactory()
        
        # Create servo object with correct pulse widths for FS90
        # min_pulse_width: 1ms (0°)
        # max_pulse_width: 2ms (180°)
        self.servo = Servo(
            pin, 
            min_pulse_width=1/1000, 
            max_pulse_width=2/1000,
            pin_factory=factory
        )
        
        # Set to center position
        self.set_angle(90)
        
    def set_angle(self, angle):
        """
        Set servo angle
        
        Args:
            angle: Angle in degrees (0-180)
        """
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        
        # Convert angle to gpiozero value (-1 to 1)
        # 0° = -1, 90° = 0, 180° = 1
        value = (angle - 90) / 90
        self.servo.value = value
        
    def center(self):
        """Move servo to center position (90°)"""
        self.servo.mid()
        
    def off(self):
        """Turn off servo"""
        self.servo.value = None
        
    def cleanup(self):
        """Cleanup servo"""
        self.servo.close()


def main():
    """Continuous PWM servo control at 50Hz"""
    
    # Initialize servo on GPIO pin 18 (Pin 12)
    servo = ServoFS90(pin=18)
    
    try:
        print("FS90 Servo Running - 50Hz PWM (gpiozero)")
        print("=" * 40)
        print("GPIO Pin: 18 (Physical Pin 12)")
        print("Press Ctrl+C to stop")
        print()
        
        # Continuously sweep the servo back and forth
        while True:
            # Sweep from 0° to 180°
            print("Sweeping 0° → 180°")
            for angle in range(0, 181, 2):
                servo.set_angle(angle)
                sleep(0.02)
            
            # Sweep from 180° back to 0°
            print("Sweeping 180° → 0°")
            for angle in range(180, -1, -2):
                servo.set_angle(angle)
                sleep(0.02)
        
    except KeyboardInterrupt:
        print("\n\nProgram stopped")
        
    finally:
        servo.cleanup()
        print("Servo cleanup complete")


if __name__ == "__main__":
    main()