from gpiozero import AngularServo 
from gpiozero.pins.lgpio import LGPIOFactory 
from time import sleep
import os 
 
class ServoTD8120MG: 
    def __init__(self, pin, min_angle=-90, max_angle=90, min_pulse_width=0.5, max_pulse_width=2.5): 
        """ 
        Initialize the TD8120MG servo with gpiozero 
         
        Args: 
            pin: GPIO pin number (BCM numbering)
            min_angle: Minimum angle (e.g., -90 for centered range)
            max_angle: Maximum angle (e.g., +90 for centered range)
            min_pulse_width: Min pulse width in milliseconds
            max_pulse_width: Max pulse width in milliseconds
        """ 
        # Use lgpio factory for Raspberry Pi 5 support
        # Software PWM has some jitter but is functional
        factory = LGPIOFactory() 
        
        self.min_angle = min_angle
        self.max_angle = max_angle
         
        # Create angular servo object with custom pulse widths for TD8120MG 
        # Centered range often gives more linear response
        self.servo = AngularServo( 
            pin, 
            min_angle=min_angle,  # Minimum angle (e.g., -90°)
            max_angle=max_angle,  # Maximum angle (e.g., +90°)
            min_pulse_width=min_pulse_width/1000,  # Min pulse
            max_pulse_width=max_pulse_width/1000,  # Max pulse
            frame_width=20/1000,  # Standard 50Hz frame (20ms) 
            pin_factory=factory 
        ) 
       
         
    def set_angle(self, angle, wait_time=0.05): 
        """ 
        Set servo angle 
         
        Args: 
            angle: Angle in degrees (min_angle to max_angle)
            wait_time: Time to wait for servo to reach position (seconds)
        """ 
        if angle < self.min_angle: 
            angle = self.min_angle
        elif angle > self.max_angle: 
            angle = self.max_angle
         
        # Directly set the angle using AngularServo 
        self.servo.angle = angle 
        # Give the servo time to reach the position 
        sleep(wait_time) 
         
    def center(self): 
        """Move servo to center position (0°)""" 
        self.servo.angle = 0 
         
    def off(self): 
        """Turn off servo""" 
        self.servo.angle = None  # Or use detach() equivalent if available 
         
    def cleanup(self): 
        """Cleanup servo""" 
        self.servo.close() 
 
 
def main(): 
    """Single sweep PWM servo control at 50Hz with configurable step size""" 
     
    # ============= CONFIGURATION ============= 
    SERVO_PHYSICAL_MIN = -90  # Servo's physical minimum angle for pulse mapping
    SERVO_PHYSICAL_MAX = 90   # Servo's physical maximum angle for pulse mapping
    
    SERVO_MIN_ANGLE = -68   # Actual minimum angle to use (avoid -85° which gpiozero mishandles)
    SERVO_MAX_ANGLE = 85    # Actual maximum angle to use (avoid extreme deadband)
    
    MIN_PULSE_WIDTH = 0.5   # Min pulse width in ms
    MAX_PULSE_WIDTH = 2.5   # Max pulse width in ms
    
    NUM_STEPS = 9           # Number of positions in sweep (reduced for larger, more consistent steps)
    # This creates 8 movements of ~19° each + 1 reset
    
    DELAY_PER_STEP = 1.5    # Delay between steps in seconds (slowed for observation)
    
    TOTAL_RANGE = SERVO_MAX_ANGLE - SERVO_MIN_ANGLE  # Total sweep range
    5
    # Wiring: Pin 2 (5V) → Servo Red, Pin 6 (GND) → Servo Brown, Pin 12 (GPIO18) → Servo Orange
    # =========================================
    
    # Initialize servo with symmetric physical range for proper pulse mapping
    servo = ServoTD8120MG(
        pin=18, 
        min_angle=SERVO_PHYSICAL_MIN,  # -90° for correct pulse width mapping
        max_angle=SERVO_PHYSICAL_MAX,  # +90° for correct pulse width mapping
        min_pulse_width=MIN_PULSE_WIDTH,
        max_pulse_width=MAX_PULSE_WIDTH
    ) 
     
    try:
        # Set higher process priority to reduce jitter
        os.nice(-10)  # Requires running with sudo for negative values
    except PermissionError:
        print("Note: Run with sudo for reduced jitter (higher process priority)")
        print()
    
    try: 
        print("\nTD8120MG Servo Running - 50Hz PWM (gpiozero)")  
        print(f"Config: PWM={MIN_PULSE_WIDTH}-{MAX_PULSE_WIDTH}ms")
        print(f"Range: {SERVO_MIN_ANGLE}° to {SERVO_MAX_ANGLE}° (total {TOTAL_RANGE}°)")
        print(f"Number of steps: {NUM_STEPS}")
        print() 
        
        # Generate evenly distributed positions from SERVO_MIN_ANGLE to SERVO_MAX_ANGLE
        positions = [int(SERVO_MIN_ANGLE + TOTAL_RANGE * i / (NUM_STEPS - 1)) for i in range(NUM_STEPS)]
        
        print(f"Command positions: {positions}")
        print()
        
        # Continuously perform single sweeps from min to max angle
        sweep_count = 0
        while True:
            # At the start of each iteration, perform full sweep
            sweep_count += 1
            print(f"--- Starting Sweep #{sweep_count} ---")
            
            # Sweep through all positions
            step_num = 0
            for i, angle in enumerate(positions): 
                servo.set_angle(angle)
                
                if i == 0:
                    # First step: large 153° reset movement from +85° to -68°
                    print(f"Sweep #{sweep_count}, Step {step_num}/{len(positions)-1} (commanded {angle}°, RESET)")
                    sleep(3.0)  # Extended time for large reset movement
                else:
                    # Regular step: ~19° incremental movement
                    print(f"Sweep #{sweep_count}, Step {step_num}/{len(positions)-1} (commanded {angle}°)")
                    sleep(DELAY_PER_STEP)  # Slowed delay for observation
                    
                step_num += 1
            
            print(f"Sweep #{sweep_count} complete.\n") 
 
         
    except KeyboardInterrupt: 
        print("\n\nProgram stopped") 
         
    finally: 
        servo.cleanup() 
        print("Servo cleanup complete") 
 
 
if __name__ == "__main__": 
    main()