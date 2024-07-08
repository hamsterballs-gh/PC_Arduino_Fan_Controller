
# 1.3 Profile Generator

# Requirements:
# Standalone code
# Profile CSV files

# Produces a full FRAM image that is compatible with 1.3.0 Fan Controller code

# Features:
# Generate location table
# Generate schedule table from CSV
# Generate gamma correction table
# Generate RGB profiles from CSV
# Generate fan profiles from CSV
# Generate FRAM strings from CSV
# Generate thermistor calibration values from CSV

import csv
import struct

software_ver = 3
data_file = "fram_gen.bin"


# UTILITY FUNCTIONS

def csv_int8(data):
    # Convert string to int8 byte
    if data:
        # return int(str(data)).to_bytes(1, byteorder='little')
        if type(data) == bytearray or type(data) == bytes:
            return data
        else:
            return int(data).to_bytes(1, byteorder='little')
    else:
        # default 0 value
        var_zero = 0
        return var_zero.to_bytes(1, byteorder='little')


def csv_int16(data):
    # Convert string to int16 bytes

    if data:
        return int(data).to_bytes(2, byteorder='little')
    else:
        # default 0 value
        var_zero = 0
        return var_zero.to_bytes(2, byteorder='little')


def csv_float(data):
    # Convert string to float bytes
    if data:
        return bytearray(struct.pack("f", float(data)))
    else:
        # default 0.0 value
        return bytearray(struct.pack("f", 0.0))


def csv_boolean(data):
    if data == "TRUE":
        var_one = 1
        return var_one.to_bytes(1, byteorder='little')
    else:
        var_zero = 0
        return var_zero.to_bytes(1, byteorder='little')


def array_denester(data):
    # Function to denest an array

    denested = []

    for i in range(len(data)):
        if (type(data[i]) == list or type(data[i]) == bytearray or type(data[i]) == bytes) and len(data[i]) > 1:
            denested_data = array_denester(data[i])
            for j in range(len(denested_data)):
                denested.append(denested_data[j])
        else:
            denested.append(data[i])

    # return sum(data, [])
    return denested


def convert_array_to_bytes(data_array):
    # Function to ensure entire array is converted to bytes
    # Expects output from array_denester, or a mixed array of single integers/bytes

    byte_array = []
    # byte_array = bytearray()

    for i in range(len(data_array)):
        try:
            byte_array.append(csv_int8(data_array[i]))
        except ValueError:
            print("Failed to convert byte!")
            byte_array.append(data_array[i])

    return byte_array


def pad_fram(data, pad_bytes):
    data += bytearray(pad_bytes)
    return data


# DATA FUNCTIONS

def make_gamma_data():
    # Creates gamma correction table
    print("Generating gamma table")
    gamma = 2.4  # Correction factor
    max_in = 4096  # Top end of INPUT range
    max_out = 4096  # Top end of OUTPUT range
    gamma_table = []

    for i in range(max_in):
        gamma_val = int(round(((i / max_in) ** gamma) * max_out + 0.5))
        gamma_table.append(gamma_val)
        # print(gamma_val)

    gamma_table_bytes = []
    print("Converting to bytes")
    for i in range(len(gamma_table)):
        int_bytes = gamma_table[i].to_bytes(2, byteorder='little')
        gamma_table_bytes.append(int_bytes[0])
        gamma_table_bytes.append(int_bytes[1])

    print("Gamma table generated:", len(gamma_table_bytes), "bytes.")

    return gamma_table_bytes


def make_rgb_data():
    # Reads 'rgb_profiles.csv' to produce profiles

    # Columns:
    # A - Profile Num, used to determine if the row contains data
    # B - Profile Name, ignored
    # C - link, int8
    # D - use_random, boolean TRUE or FALSE
    # E - smooth, boolean TRUE or FALSE
    # F - brightness, float
    # G - seq_pos, int8
    # H - seq_len, int8
    # I - rate, float
    # J - transition, float (usually set to 0.0)
    # K-AN - Red, Green, Blue columns with int16 values
    print("Generating RGB profile data")

    profile_bytes = []
    profiles_read = 0

    with open("fan_controller_rgb_profiles.csv") as csv_file:
        reader = csv.reader(csv_file)

        for row in reader:
            # for i in row:
            # 	print(i, type(i))

            if row[0] and row[0] != "Profile Num":
                profiles_read += 1

                # link
                profile_bytes.append(csv_int8(row[2]))

                # use_random
                profile_bytes.append(csv_boolean(row[3]))

                # smooth
                profile_bytes.append(csv_boolean(row[4]))

                # brightness
                profile_bytes.append(csv_float(row[5]))

                # seq_pos
                profile_bytes.append(csv_int8(row[6]))

                # seq_len
                profile_bytes.append(csv_int8(row[7]))

                # rate
                profile_bytes.append(csv_float(row[8]))

                # transition
                profile_bytes.append(csv_float(row[9]))

                # RGB
                for i in range(30):
                    profile_bytes.append(csv_int16(row[10 + i]))

    profile_bytes = convert_array_to_bytes(array_denester(profile_bytes))

    print("RGB profile data created:", profiles_read, "profiles,", len(profile_bytes), "bytes.")

    # print(profile_bytes)

    return profiles_read, profile_bytes


def make_fan_data():
    # Reads 'fan_profiles.csv' to produce profiles

    # Columns:
    # A - Profile Num, used to determine if the row contains data
    # B - Profile Name, ignored
    # C - pwm, boolean TRUE or FALSE
    # D - allow_off, boolean TRUE or FALSE
    # E - max_voltage, int16
    # F - changerate, int8
    # G - hysteresis, int8
    # H - last_temperature, int8
    # I - last_fan_speed, int16
    # J - target_fan_speed, int16
    # K - default_temp, int8
    # L-S - factor, int8
    # T-W - temperature, int8
    # X-AA - fan_speed, int16

    print("Generating fan profile data")

    profile_bytes = []
    profiles_read = 0

    with open("fan_controller_fan_profiles.csv") as csv_file:
        reader = csv.reader(csv_file)

        for row in reader:
            # for i in row:
            # 	print(i, type(i))

            if row[0] and row[0] != "Profile Num":
                profiles_read += 1

                # pwm
                profile_bytes.append(csv_boolean(row[2]))

                # allow_off
                profile_bytes.append(csv_boolean(row[3]))

                # max_voltage
                profile_bytes.append(csv_int16(row[4]))

                # changerate
                profile_bytes.append(csv_int8(row[5]))

                # hysteresis
                profile_bytes.append(csv_int8(row[6]))

                # last_temperature
                profile_bytes.append(csv_int8(row[7]))

                # last_fan_speed
                profile_bytes.append(csv_int16(row[8]))

                # target_fan_speed
                profile_bytes.append(csv_int16(row[9]))

                # default_temp
                profile_bytes.append(csv_int8(row[10]))

                # factors
                for i in range(8):
                    profile_bytes.append(csv_int8(row[11 + i]))

                # temperatures
                # fan_speed
                for i in range(4):
                    profile_bytes.append(csv_int8(row[19 + i]))
                    profile_bytes.append(csv_int16(row[23 + i]))

    # profile_bytes = convert_array_to_bytes(array_denester(profile_bytes))
    profile_bytes = array_denester(profile_bytes)

    print("Fan profile data created:", profiles_read, "profiles -", len(profile_bytes), "bytes.")

    # print(profile_bytes)

    return profiles_read, profile_bytes


def make_fram_strings():
    # Function to make table of FRAM strings and null pad with bytes
    # Set maximum string length below:
    max_string_length = 16

    # Reads strings from column 0 of CSV

    print("Generating FRAM string data")

    fram_string_bytes = []
    strings_read = 0

    with open("fan_controller_strings.csv") as csv_file:
        reader = csv.reader(csv_file)

        for row in reader:
            strings_read += 1
            string_bytes = bytearray(row[0].encode('ascii'))

            if len(string_bytes) > max_string_length:
                print("Warning!", row[0], "greater than max length!")

            string_bytes = string_bytes[0:max_string_length]

            for i in range(max_string_length - len(string_bytes)):
                string_bytes += b'\0'

            fram_string_bytes.append(string_bytes)

    fram_string_bytes = array_denester(fram_string_bytes)

    print("FRAM string data generated:", strings_read, "strings -", len(fram_string_bytes), "bytes.")

    return strings_read, max_string_length, fram_string_bytes


def make_calibrations():
    # Function to get calibration values for thermistors and LDR

    # CSV layout:
    # Row 0 - header
    # Row 1-4 - thermistor 1-4 values, A = description, B = low reading, C = low value, D = high reading, E = high value
    # Row 5 - LDR values, same as above

    print("Generating calibration data")

    cal_bytes = []

    with open("fan_controller_calibration.csv") as csv_file:
        csv_contents = []

        reader = csv.reader(csv_file)

        for row in reader:
            csv_contents.append(row)

        for cal in range(5):
            for value in range(4):
                cal_bytes.append(csv_int16(csv_contents[cal + 1][value + 1]))

    cal_bytes = array_denester(cal_bytes)

    # print(cal_bytes)

    print("Calibration data generated:", len(cal_bytes), "bytes.")

    return cal_bytes


def make_lookup_table():
    # Function to create the lookup table for various bits of data

    print("Generating lookup table")

    data_start = 200
    data_address = data_start

    # Where previously generated data arrays are put in, after the initial config space

    table_bytes = []

    table_bytes.append(software_ver.to_bytes(2, byteorder='little'))
    table_bytes = pad_fram(table_bytes, 6)

    # 8-9 - schedule table
    table_bytes = pad_fram(table_bytes, 2)

    # 10-11 - gamma correction table
    table_bytes.append(csv_int16(data_address))

    # 12-13 - rgb profiles location
    data_address += len(gamma_data)
    table_bytes.append(csv_int16(data_address))
    # 14-14 - number of rgb profiles
    table_bytes.append(csv_int8(rgb_profiles))

    # 15-16 - fan profiles location
    data_address += len(rgb_data)
    table_bytes.append(csv_int16(data_address))
    # 17-17 - number of fan profiles
    table_bytes.append(csv_int8(fan_profiles))

    # 18-19 - fram strings location
    data_address += len(fan_data)
    table_bytes.append(csv_int16(data_address))
    # 20-20 - string length
    table_bytes.append(csv_int8(string_length))
    # 21-22 - number of fram strings
    table_bytes.append(csv_int16(string_profiles))

    # 23-25 - default rgb profiles (int8)
    table_bytes.append(csv_int8(25))  # Custom RGB channels, day/night not currently implemented
    table_bytes.append(csv_int8(26))
    table_bytes.append(csv_int8(27))

    # 26-31 - default fan profiles (int8)
    table_bytes.append(csv_int8(0))
    table_bytes.append(csv_int8(1))
    table_bytes.append(csv_int8(2))
    table_bytes.append(csv_int8(3))
    table_bytes.append(csv_int8(4))
    table_bytes.append(csv_int8(5))

    # 32-47 - thermistor calibration values (2xint16 per thermistor)
    # 48-51 - LDR calibration values (2xint16)
    table_bytes.append(calibration_data)

    # Add padding to get to the data_address

    table_bytes = array_denester(table_bytes)
    # for i in range(len(table)):
    # 	print(table[i])

    # print(table_bytes)

    bytes_to_pad = data_start - len(table_bytes)
    # print("Padding", bytes_to_pad, "bytes")
    table_bytes = pad_fram(table_bytes, bytes_to_pad)

    # print(table)

    print("Lookup table generated -", len(table_bytes), "bytes.")

    return table_bytes


def make_bin():
    # Function to combine the files and tables to produce a 32KB FRAM image

    print("Making FRAM image")

    # schedule
    # gamma
    # rgb
    # fan
    # fram

    bin_bytes = lookup_data
    bin_bytes = array_denester(bin_bytes)

    print("Gamma table location:", str(len(bin_bytes)))
    bin_bytes.append(gamma_data)
    bin_bytes = array_denester(bin_bytes)

    print("RGB data location:", str(len(bin_bytes)))
    bin_bytes.append(rgb_data)
    bin_bytes = array_denester(bin_bytes)

    print("Fan data location:", str(len(bin_bytes)))
    bin_bytes.append(fan_data)
    bin_bytes = array_denester(bin_bytes)

    print("String data location:", str(len(bin_bytes)))
    bin_bytes.append(string_data)
    bin_bytes = array_denester(bin_bytes)

    print("Data size is", len(bin_bytes), "bytes.")

    pad_amount = 32768 - len(bin_bytes)
    bin_bytes = pad_fram(bin_bytes, pad_amount)

    bin_bytes = convert_array_to_bytes(bin_bytes)
    # print(bin_bytes)

    try:
        with open(data_file, "wb") as file:
            for i in range(len(bin_bytes)):
                file.write(bin_bytes[i])
            # file.write(bin_bytes)

        print("File written.")
    except OSError:
        print("Unable to open file - is it open?")


def verify_file():
    # Function to check that a section of bytes are saved correctly
    # Prints out byte values.
    print("Verify test")
    with open(data_file, "rb") as file:
        data = file.read()

    print(data[11164:11196])

    for i in range(32):
        print(int(data[11164 + i]))


if __name__ == "__main__":
    print("1.3 profile generator")

    gamma_data = make_gamma_data()
    rgb_profiles, rgb_data = make_rgb_data()
    fan_profiles, fan_data = make_fan_data()
    string_profiles, string_length, string_data = make_fram_strings()
    calibration_data = make_calibrations()

    lookup_data = make_lookup_table()

    make_bin()

    # verify_file()
