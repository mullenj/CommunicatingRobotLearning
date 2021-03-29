import serial
import platform

commands = {'all_on': 'A', 'left_on': 'L', 'right_on': 'R', 'top_on': 'T', 'bottom_on': 'B', 'tense': 'O', 'relax': 'P', 'circular': 'C', 'striped': 'S', 'LED': 'H'}

def initialize():
    # Define COM Serial Port
    if platform.system() == 'Windows':
        COM_PORT = 'COM9'
    elif platform.system() == 'Linux':
        COM_PORT = '/dev/rfcomm0'

    # Initiate Bluetooth Connection
    bluetooth = serial.Serial(COM_PORT, 115200)
    print("[*] Connected to Haptic Device")
    return bluetooth

# Command List
# 'all_on' - All vibrotactors on
# 'left' - Left 2 vibrotactors on
# 'right' - Right 2 vibrotactors on
# 'top' - Top 2 on
# 'bottom' - Bottom 2 on
# 'tense' - Increase tension for 1s
# 'relax' - Decrease tension for 1s
# 'circular' - Circular vibrotactor pattern (Set duration for # of cycles) !Not implemented in arduino
# 'striped' - On/Off alternating vibrotactor pattern (Set duration for # of cycles) !Not implemented in arduino
# 'LED' - Turns LED_BUILTIN on

def haptic_command(conn, command, PW, duration):
    commands = {'all_on': 'A', 'left_on': 'L', 'right_on': 'R', 'top_on': 'T', 'bottom_on': 'B', 'tense': 'O', 'relax': 'P', 'circular': 'C', 'striped': 'S', 'LED': 'H'}
    if command in commands.keys() and PW >= 0 and PW < 10: # Check Input
        conn.write(str.encode(commands[command] + str(PW) + str(duration) + '\n')) # Send Command

def close(conn):
    conn.close()

# Test Code
# ---------
def main():
    conn = initialize()
    while True:
        print("Type 'Quit' to quit")
        print(commands)
        user_input = input(" >>  ")
        if user_input == 'Quit':
            break
        haptic_command(conn, user_input, 3, 5)
    close(conn)


if __name__ == "__main__":
    main()
