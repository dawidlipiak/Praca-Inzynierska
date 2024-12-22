import hid
import time

def print_hex_and_ascii(data):
    # Print hex values
    hex_values = ' '.join([f'{x:02X}' for x in data])
    
    # Convert to ASCII (only printable characters, others shown as '.')
    ascii_values = ''.join([chr(x) if 32 <= x <= 126 else '.' for x in data])
    
    # Reconstruct 32-bit value from first 4 bytes (little-endian)
    uint32_value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
    
    print(f"HEX: {hex_values}")
    print(f"ASCII: {ascii_values}")
    print(f"32-bit value: {uint32_value} (0x{uint32_value:08X})")

def main():
    # VID and PID for your device
    VENDOR_ID = 1155      # 1155 in decimal
    PRODUCT_ID = 22352    # 22352 in decimal
    ################################################## ZARAZ WRACAM ################################################
    try:
        # Open the HID device
        device = hid.device()
        device.open(VENDOR_ID, PRODUCT_ID)
        
        print(f"Successfully opened HID device:")
        print(f"Manufacturer: {device.get_manufacturer_string()}")
        print(f"Product: {device.get_product_string()}")
        print(f"Serial Number: {device.get_serial_number_string()}")
        
        # Prepare and send report with value 1
        report = [0] * 64  # Create 64-byte report filled with zeros
        report[1] = 1      # Set first byte to 1 , index 0 is report id
        
        print("\nSending report with value 1...")
        device.write(report)
        print("Report sent successfully")
        
        print("\nWaiting for response... Press Ctrl+C to exit.\n")
        
        # Continuously read reports
        while True:
            try:
                # Read 64 bytes report
                response = device.read(64)
                if response:
                    # Print timestamp and data
                    print(f"\n[{time.strftime('%H:%M:%S')}] Received:")
                    print_hex_and_ascii(response)
                    
            except IOError as e:
                print(f"Error reading from device: {e}")
                break
                
    except IOError as e:
        print(f"Error opening device: {e}")
        print("Make sure the device is connected and you have the right permissions")
        
    except KeyboardInterrupt:
        print("\nExiting...")
        
    finally:
        # Make sure to close the device
        if 'device' in locals():
            device.close()

if __name__ == "__main__":
    main()