
# 1.3 Profile Updater

# Update a section of FRAM, such as fan, RGB or string data

# Requirements:
# 1.3 Profile Generator
# Profile CSV files

import serial_130 as fc
import profile_generator_130 as pg


def update_rgb():
	read_profiles_loc = fc.cmd_read_fram(12, 2)
	read_profiles_loc = int.from_bytes(read_profiles_loc, byteorder='little')
	print("RGB profiles address:", read_profiles_loc)

	read_profiles_num = fc.cmd_read_fram(14, 1)
	read_profiles_num = int.from_bytes(read_profiles_num, byteorder='big')
	print("Number of RGB profiles:", read_profiles_num)

	gen_rgb_profiles, gen_rgb_bytes = pg.make_rgb_data()

	if gen_rgb_profiles != read_profiles_num:
		print("Number of RGB profiles do not match!")
		return

	print("Updating RGB profiles")
	fc.cmd_write_fram(read_profiles_loc, gen_rgb_bytes)
	print("Fan profiles updated")


def update_fans():
	read_profiles_loc = fc.cmd_read_fram(15, 2)
	read_profiles_loc = int.from_bytes(read_profiles_loc, byteorder='little')
	print("Fan profiles address:", read_profiles_loc)

	read_profiles_num = fc.cmd_read_fram(17, 1)
	read_profiles_num = int.from_bytes(read_profiles_num, byteorder='big')
	print("Number of fan profiles:", read_profiles_num)

	gen_fan_profiles, gen_fan_bytes = pg.make_fan_data()

	print(gen_fan_bytes)



	if gen_fan_profiles != read_profiles_num:
		print("Number of fan profiles do not match!")
		return

	print("Updating fan profiles")
	fc.cmd_write_fram(read_profiles_loc, gen_fan_bytes)
	print("Fan profiles updated")


def update_strings():
	read_strings_loc = fc.cmd_read_fram(18, 2)
	read_strings_loc = int.from_bytes(read_strings_loc, byteorder='little')
	print("Strings table address:", read_strings_loc)

	read_strings_length = fc.cmd_read_fram(20, 1)
	read_strings_length = int.from_bytes(read_strings_length, byteorder='big')
	print("String length:", read_strings_length)

	read_strings_num = fc.cmd_read_fram(21, 1)
	read_strings_num = int.from_bytes(read_strings_num, byteorder='big')
	print("Number of strings:", read_strings_num)

	# strings_read, max_string_length, fram_string_bytes
	gen_strings_num, gen_strings_length, gen_strings_bytes = pg.make_fram_strings()

	if gen_strings_num != read_strings_num or gen_strings_length != read_strings_length:
		print("String number and length do not match!")
		return

	print("Updating string table")
	fc.cmd_write_fram(read_strings_loc, gen_strings_bytes)
	print("String table updated")


if __name__ == "__main__":
	print("Profile Updater")

	# update_rgb()
	update_fans()
	# update_strings()

	# fc.cmd_load_config()

	print("Done!")
