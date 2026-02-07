from machine import UART, Pin
import time
from ld2410 import LD2410

# Configuration for Raspberry Pi Pico (RP2040)
# Using UART1 on GPIO8 (TX) and GPIO9 (RX)
UART_ID = 1
TX_PIN = 8
RX_PIN = 9
BAUDRATE = 256000

def main():
    print("Initializing LD2410 on UART{} (TX={}, RX={})".format(UART_ID, TX_PIN, RX_PIN))
    
    try:
        uart = UART(UART_ID, baudrate=BAUDRATE, tx=Pin(TX_PIN), rx=Pin(RX_PIN))
    except Exception as e:
        print("Failed to init UART:", e)
        return

    sensor = LD2410(uart)
    
    print("Reading sensor data. Press Ctrl+C to stop.")
    
    # Optional: Enter config mode to read firmware version
    try:
        sensor.enter_config_mode()
        print(" entered config mode")
        
        # Firmware version is often 8 bytes
        # version = sensor.read_firmware_version()
        # print("Firmware Version (raw bytes):", version)
        
        sensor.exit_config_mode()
        print(" exited config mode")
    except Exception as e:
        print("Warning: Could not read firmware version or enter config mode:", e)

    while True:
        try:
            # Blocks until a valid frame is received
            data = sensor.read_presence()
            
            if data:
                status = "None"
                if data.target_state == 1: status = "Moving"
                elif data.target_state == 2: status = "Stationary"
                elif data.target_state == 3: status = "Both"
                
                print(f"Status: {status:10} | Dist: {data.detection_distance_cm}cm")
                print(f"  Mov: {data.moving_distance_cm}cm (Energy: {data.moving_energy})")
                print(f"  Sta: {data.still_distance_cm}cm (Energy: {data.still_energy})")
                print("-" * 40)
                
            time.sleep(0.1) 
            
        except KeyboardInterrupt:
            print("Stopping...")
            break
        except Exception as e:
            print("Error:", e)
            time.sleep(1)

if __name__ == "__main__":
    main()
