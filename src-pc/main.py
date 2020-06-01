from numpy import interp
from time import sleep
import serial
import inputs
import threading


gamepad_states = {
    "BTN_SELECT": 0,
    "BTN_START": 0,
    "BTN_NORTH": 0,
    "BTN_SOUTH": 0,
    "BTN_EAST": 0,
    "BTN_WEST": 0,
    "BTN_TL": 0,
    "BTN_TR": 0,
    "BTN_THUMBL": 0,
    "BTN_THUMBR": 0,
    "ABS_Z": 0,
    "ABS_RZ": 0,
    "ABS_X": 0,
    "ABS_Y": 0,
    "ABS_RX": 0,
    "ABS_RY": 0,
    "ABS_HAT0X": 0,
    "ABS_HAT0Y": 0,
}


def eventloop():
    global gamepad_states
    while True:
        events = inputs.get_gamepad()
        for event in events:
            if event.code in gamepad_states:
                gamepad_states[event.code] = event.state


# bluetooth
try:
    ser = serial.Serial("COM3", 38400, timeout=0, parity=serial.PARITY_EVEN, rtscts=1)
except serial.SerialException:
    print("error: no bluetooth port found")
    # exit()

# gamepad
pads = inputs.devices.gamepads
if len(pads) == 0:
    print("error: gamepad not found")
    exit()
threading.Thread(target=eventloop).start()


# main program
leftaccum = 0
rightaccum = 0
while True:
    sleep(0.05)

    gas = interp(gamepad_states["ABS_RZ"], [0, 255], [0, 90]) - interp(
        gamepad_states["ABS_Z"], [0, 255], [0, 90]
    )

    wheel = gamepad_states["ABS_X"]
    if wheel < 5000 and wheel > -5000:
        wheel = 0
    wheel = interp(wheel, [-32768, 32767], [-30, 30])

    leftaccum = int(0.7 * leftaccum + 0.3 * (gas + wheel))
    rightaccum = int(0.7 * rightaccum + 0.3 * (gas - wheel))

    print(f"LEFT:{leftaccum}\tRIGHT:{rightaccum}")
    ser.write(f"DRIVE:{leftaccum},{rightaccum}#".encode("ASCII"))


# ser.write(b"DRIVE:90,90#")
# sleep(1)
# ser.write(b"DRIVE:-50,50#")
# sleep(2)

# ser.write(b"DRIVE:90,90#")
# sleep(1)
# ser.write(b"DRIVE:-50,50#")
# sleep(2)

# ser.write(b"DRIVE:90,90#")
# sleep(1)
# ser.write(b"DRIVE:-50,50#")
# sleep(2)

# ser.write(b"DRIVE:90,90#")
# sleep(1)
# ser.write(b"DRIVE:-50,50#")
# sleep(2)

# ser.write(b"KILL#")
