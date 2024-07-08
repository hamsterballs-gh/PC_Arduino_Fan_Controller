# LCD_Strings_130

# Reads a 2 column CSV file
# Column 1 is LCD line 1, Column 2 is LCD line 2, Column 3 is delay to next row in seconds
# Cycles through each row continuously

import serial_130 as fc
import csv
import time

strings_csv = "lcd_strings.csv"


def read_data(file_name):
    print("Reading file:", file_name)

    csv_data = []

    with open(file_name) as csv_file:
        reader = csv.reader(csv_file)

        for row in reader:
            csv_data.append([row[0], row[1], int(row[2])])

    return csv_data


def update_lcd(timeout, string_1, string_2):
    fc.cmd_write_lcd(timeout, string_1, string_2)
    # pass


if __name__ == "__main__":
    strings_data = read_data(strings_csv)

    print("Strings found:", len(strings_data))
    print("Commencing...")

    while True:
        for entry in strings_data:
            print(entry[2] + 3, "|", entry[0], "|", entry[1])
            update_lcd(entry[2] + 3, entry[0], entry[1])

            if entry[2]:
                time.sleep(entry[2])
            else:
                time.sleep(1)

