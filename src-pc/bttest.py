import bluetooth

bd_addr = "98:D3:31:20:94:37"

port = 1

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr, port))

print("connected, now sending")
sock.send("BEEP:20,20#")
print("sent, now closing.")

sock.close()


# # simple inquiry example
# import bluetooth

# nearby_devices = bluetooth.discover_devices(lookup_names=True)
# print("Found {} devices.".format(len(nearby_devices)))

# for addr, name in nearby_devices:
#     print("  {} - {}".format(addr, name))
