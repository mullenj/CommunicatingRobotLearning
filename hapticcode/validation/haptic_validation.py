import serial
import platform
# import time
# import threading

commands = {'ten': 'A', 'twenty': 'L', 'thirty': 'R', 'forty': 'T', 'fifty': 'B', 'sixty': 'O', 'relax': 'P', 'seventy': 'C', 'eighty': 'S','ninety': 'N','hundred': 'H', 'LED': 'H'}
def initialize():
    # Define COM Serial Port
    if platform.system() == 'Windows':
        COM_PORT = 'COM12'
    elif platform.system() == 'Linux':
        COM_PORT = '/dev/rfcomm0'

    # Initiate Bluetooth Connection
    bluetooth = serial.Serial(COM_PORT, 115200)
    print("[*] Connected to Haptic Device")
    return bluetooth

# Command List
# 'ten' - 10% PWM
# 'twenty' - 20%
# 'thirty' - 30%
# 'forty' - 40%
# 'fifty' - 50%
# 'sixty' - 60%
# 'relax' - Decrease tension for 1s
# 'seventy' - 70%
# 'eighty' - 80%
# 'ninety' - 90%
# 'hundred' - 100%
# 'LED' - Turns LED_BUILTIN on

def haptic_command(conn, command, PW, duration):
    commands = {'ten': 'A', 'twenty': 'L', 'thirty': 'R', 'forty': 'T', 'fifty': 'B', 'sixty': 'O', 'relax': 'P', 'seventy': 'C', 'eighty': 'S','ninety': 'N','hundred': 'H', 'LED': 'H'}
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
        if user_input == 'tense' or user_input == 'relax':
            haptic_command(conn, user_input, 3, 1)
        else:
            haptic_command(conn, user_input, 1, 1)            
    close(conn)


if __name__ == "__main__":
    main()
