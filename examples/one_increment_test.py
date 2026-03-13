from gpiozero import AngularServo 
from gpiozero.pins.lgpio import LGPIOFactory 
from time import sleep
import os 

class ServoTD8120MG: 
    def __init__(self, pin, min_angle=-90, max_angle=90, min_pulse_width=0.5, max_pulse_width=2.5): 
        """ 
        Initialize the TD8120MG servo with gpiozero 
        Uses AngularServo for precise angle control
        """ 
        factory = LGPIOFactory() 
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 0  # Track current operational angle
         
        # Create angular servo object with custom pulse widths 
        self.servo = AngularServo( 
            pin, 
            min_angle=min_angle,  # Physical minimum (-90°)
            max_angle=max_angle,  # Physical maximum (+90°)
            min_pulse_width=min_pulse_width/1000,  # Min pulse (0.5ms)
            max_pulse_width=max_pulse_width/1000,  # Max pulse (2.5ms)
            frame_width=20/1000,  # Standard 50Hz frame (20ms) 
            pin_factory=factory 
        ) 
       
    def set_angle(self, angle, wait_time=0.05): 
        """ 
        Set servo angle with bounds checking
        """
        # Clamp to physical limits
        if angle < self.min_angle: 
            angle = self.min_angle
        elif angle > self.max_angle: 
            angle = self.max_angle
         
        self.servo.angle = angle 
        self.current_angle = angle
        sleep(wait_time) 
         
    def get_current_angle(self):
        """Get current operational angle"""
        return self.current_angle
         
    def center(self): 
        """Move servo to center position (0°)""" 
        self.servo.angle = 0 
        self.current_angle = 0
         
    def off(self): 
        """Turn off servo""" 
        self.servo.angle = None  # Or use detach() equivalent if available 
         
    def cleanup(self): 
        """Cleanup servo""" 
        self.servo.close() 


def main():
    """Servo stepping controller with manual commands for precise control"""
    
    # Configuration - step size configurable only via code
    SERVO_PHYSICAL_MIN = -90    # Physical range: -90°
    SERVO_PHYSICAL_MAX = 90     # Physical range: +90°
    OPERATIONAL_MIN = -75       # Operational range: start at -68°
    OPERATIONAL_MAX = 87        # Operational range: end at +85°
    STARTING_POSITION = OPERATIONAL_MIN  # Start at operational minimum (-68°)
    STEP_SIZE = 18              # Configurable only via code (fine control)
    STEP_DELAY = 0.05           # Delay between steps (seconds)
    
    print("=== Servo Precise Stepping Test ===")
    print(f"Physical range: {SERVO_PHYSICAL_MIN}° to {SERVO_PHYSICAL_MAX}°")
    print(f"Operational range: {OPERATIONAL_MIN}° to {OPERATIONAL_MAX}°")
    print(f"Starting position: {STARTING_POSITION}°")
    print(f"Step size: {STEP_SIZE}° (configurable only in code)")
    print("Commands:")
    print("  y - Step once (move {STEP_SIZE}° forward)")
    print("  n - Step once (move {STEP_SIZE}° backward)") 
    print("  r - Reset to starting position")
    print("  q - Quit gracefully")
    print()
    
    # Initialize servo
    servo = ServoTD8120MG(
        pin=18, 
        min_angle=SERVO_PHYSICAL_MIN,  # -90° physical limit
        max_angle=SERVO_PHYSICAL_MAX,  # +90° physical limit
        min_pulse_width=0.5,           
        max_pulse_width=2.5            
    )
    
    # Set initial position
    servo.set_angle(STARTING_POSITION)
    current_pos = STARTING_POSITION
    print(f"Initialized at {current_pos}°")
    
    # Set higher process priority to reduce jitter
    try:
        os.nice(-10)
    except PermissionError:
        print("Note: Run with sudo for reduced jitter (higher process priority)")
        print()

    def step_once(direction=1):
        """Function that steps the servo by current step size in specified direction"""
        nonlocal current_pos
        
        # Calculate next position
        new_position = current_pos + (STEP_SIZE * direction)
        
        # Wrap around: if we go beyond +90°, wrap to -90°, and vice versa
        if new_position > SERVO_PHYSICAL_MAX:
            new_position = SERVO_PHYSICAL_MIN + (new_position - SERVO_PHYSICAL_MAX - 1)
            print(f"Wrapping from +90° to -90°")
        elif new_position < SERVO_PHYSICAL_MIN:
            new_position = SERVO_PHYSICAL_MAX - (SERVO_PHYSICAL_MIN - new_position - 1)
            print(f"Wrapping from -90° to +90°")
        
        # Move to new position
        servo.set_angle(new_position, wait_time=STEP_DELAY)
        current_pos = new_position
        
        # Show operational vs physical context
        if OPERATIONAL_MIN <= current_pos <= OPERATIONAL_MAX:
            op_status = "OPERATIONAL"
        else:
            op_status = "BEYOND_OPERATIONAL"
            
        direction_str = "+" if direction == 1 else "-"
        print(f"Stepped to {current_pos}° (change: {direction_str}{STEP_SIZE}°) [{op_status}]")
        return True

    def reset_to_start():
        """Reset servo to starting position"""
        nonlocal current_pos
        servo.set_angle(STARTING_POSITION, wait_time=0.5)
        current_pos = STARTING_POSITION
        
        # Show operational vs physical context
        if OPERATIONAL_MIN <= current_pos <= OPERATIONAL_MAX:
            op_status = "OPERATIONAL"
        else:
            op_status = "BEYOND_OPERATIONAL"
            
        print(f"Reset to starting position: {current_pos}° [{op_status}]")

    try:
        step_count = 0
        while True:
            print(f"\nCurrent: {current_pos}° | Steps: {step_count} | Commands: y/n(step), r(reset), q(quit)")
            user_input = input("Enter command: ").strip().lower()
            
            if user_input == 'y':  # Forward step
                step_once(direction=1)
                step_count += 1
            elif user_input == 'n':  # Backward step
                step_once(direction=-1)
                step_count += 1
            elif user_input == 'r':  # Reset
                reset_to_start()
                step_count = 0
            elif user_input == 'q':  # Quit
                print(f"Final stats - Total steps: {step_count}, Final position: {current_pos}°")
                print("Quitting gracefully...")
                break
            else:
                print("Invalid command. Use: y/n(step), r(reset), q(quit)")
    
    except KeyboardInterrupt:
        print(f"\n\nInterrupted - Total steps: {step_count}, Final position: {current_pos}°")
    
    finally:
        # Return to safe position before cleanup
        servo.center()
        servo.cleanup()
        print("Servo cleanup complete")


if __name__ == "__main__":
    main()