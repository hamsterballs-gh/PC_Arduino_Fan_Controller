
# Fram dumper
# Code to read all FRAM from Fan Controller

import serial_130 as fc

dump_name = "fc_fram.bin"
write_name = "fram_gen.bin"


def dump():
	print("Dumping FRAM")
	dump_data = fc.cmd_read_fram(0, 32768)

	if dump_data:
		print("Data received, saving to file...")
		with open(dump_name, "wb") as file:
			file.write(bytes(dump_data))
	else:
		print("Dump failed!")


def write():
	print("Writing image")

	# with open('Generator/fram.bin', 'rb') as file:
	with open(write_name, 'rb') as file:
		data = file.read()

	if fc.cmd_write_fram(0, data):
		print("Write successful.")
	else:
		print("Write failed!")


# Temporary code to test config
def load_config():
	# Trigger config, then print out everything that gets sent back
	fc.cmd_load_config()
	# Waiting 2 seconds should be enough time for everything to be sent
	fc.wait(5)


if __name__ == "__main__":
	# fc.fast_wait = 0.0  # Go really fast when serial is blocking, no longer needed
	print("Fan Controller FRAM Dumper")
	fc.wait(3)
	print("Now commencing")

	# dump()
	write()
	load_config()

	print("Done")
