#!/bin/python3
import pexpect
from getpass import getpass
from time import sleep

passw = getpass("podaj hasło sudo: ")

print("turning on bluetooth")
proc = pexpect.spawn("sudo rfkill unblock 1 2")
proc.expect("sudo")
proc.sendline(passw)
proc.expect(pexpect.EOF)
print("bluetooth on")

print("searching for device")
proc = pexpect.spawn("bluetoothctl")
proc.expect("Agent registered.*#")
proc.sendline("agent KeyboardOnly")
proc.expect("#")
proc.sendline("default-agent")
proc.expect("#")
proc.sendline("power on")
proc.expect("#")
print("powered on (probably)")
proc.sendline("scan on") # if returns notready then throw error
print("scanning")
proc.expect("98:D3:31:20:94:37.*#")
print("device found")

print("pairing")
proc.sendline("pair 98:D3:31:20:94:37")
i = proc.expect(["Enter PIN code:", "AlreadyExists"])
if i == 0:
    proc.sendline("1234")
    proc.expect("#")
    proc.sendline("exit")
    proc.expect(pexpect.EOF)
    print("device pairing done")
elif i == 1:
    print("device already paired")


print("binding rfcomm")
proc = pexpect.spawn("sudo rfcomm bind 0 98:D3:31:20:94:37")
proc.expect("sudo")
proc.sendline(passw)

i = proc.expect([pexpect.EOF,"already in use"])
if i == 0:
    print("rfcomm bound")
elif i == 1:
    print("rfcomm already bound")






