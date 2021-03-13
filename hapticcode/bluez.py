import bluetooth
import serial
import time

bluetooth = serial.Serial('/dev/tty0', 115200)
print("Connected")

def led_on_off():
    user_input = input("\n>>Type H / L / S / P / Y / Q for quit : ")
    if user_input =="H":
        print("1st.")
        time.sleep(0.1)
        bluetooth.write(b'H')
        led_on_off()
    elif user_input =="L":
        print("2nd")
        time.sleep(0.1)
        bluetooth.write(b'L')
        led_on_off()
    elif user_input =="S":
        print("3rd")
        time.sleep(0.1)
        bluetooth.write(b'S')
        led_on_off()
    elif user_input =="P":
        print("4th")
        time.sleep(0.1)
        bluetooth.write(b'P')
        led_on_off()
    elif user_input =="Y":
        print("5th")
        time.sleep(0.1)
        bluetooth.write(b'Y')
        led_on_off()
    elif user_input =="U":
        print("6th")
        time.sleep(0.1)
        bluetooth.write(b'U')
        led_on_off()
    elif user_input =="T":
        print("7th")
        time.sleep(0.1)
        bluetooth.write(b'T')
        led_on_off()
    elif user_input =="F":
        print("8th")
        time.sleep(0.1)
        bluetooth.write(b'F')
        led_on_off()
    elif user_input =="R":
        print("9th")
        time.sleep(0.1)
        bluetooth.write(b'R')
        led_on_off()
    elif user_input =="Q" :
        print("Program Exiting")
        time.sleep(0.1)
        bluetooth.write(b'Q')
        bluetooth.close()
    else:
        print("Invalid input. Type H / L / S / P / Y / Q for quit.")
        led_on_off()


time.sleep(2) # wait for the serial connection to initialize

led_on_off()
