import bluetooth
import serial
import time

bluetooth = serial.Serial('/dev/rfcomm0', 115200) #COM DIFFERENT
print("Connected")

def led_on_off():
    print('>> H: vibrotactor (0)')
    print('>> S: vibrotactor (1)')
    print('>> P: vibrotactor (2)')
    print('>> Y: vibrotactor (3)')
    print('>> U: vibrotactor (4)')
    print('>> T: vibrotactor (5)')
    print('>> F: Releasing')
    print('>> R: Squeezing')
    print('>> L: Everything OFF')
    print('>> Q: Quit the program')
    user_input = input(" >>  ")
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
    elif user_input =="K":
        print("7th")
        time.sleep(0.1)
        bluetooth.write(b'K')
        led_on_off()
    elif user_input =="J":
        print("7th")
        time.sleep(0.1)
        bluetooth.write(b'J')
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
