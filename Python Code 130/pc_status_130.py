# PC_Status_130
import GPUtil

# Displays PC status information

import serial_130 as fc
import time
from datetime import datetime
import platform
import psutil
import GPUtil

display_speed = 7  # How many seconds for each status to display


def update_lcd(display_time, string_1, string_2):
    # serial_130 will automatically pad strings and cut themn off if they are too long
    # lcd timeout is specified in seconds
    # However, we will scroll any strings that are too long on string_2 (LCD bottom row)
    # We will also center string_1 if possible

    # Calculating centering for string_1
    offset = int((16 - len(string_1)) / 2)
    string_1 = (" " * offset) + string_1

    # Calculating scrolling for string_2
    if len(string_2) > 16:
        times_to_scroll = len(string_2) - 15
    else:
        times_to_scroll = 1

    # Update the LCD
    for i in range(times_to_scroll):
        fc.cmd_write_lcd(display_time + 1, string_1, string_2[i:])
        time.sleep(display_time / times_to_scroll)


#################
# Display Modes #
#################

def get_system_uname():
    update_lcd(display_speed, "System:", platform.uname().system)
    update_lcd(display_speed, "Hostname:", platform.uname().node)
    update_lcd(display_speed, "Release:", platform.uname().release)
    update_lcd(display_speed, "Version:", platform.uname().version)
    update_lcd(display_speed, "Architecture:", platform.uname().machine)
    update_lcd(display_speed, "Processor:", platform.uname().processor)


def get_boot_time():
    boot_timestamp = psutil.boot_time()
    bt = datetime.fromtimestamp(boot_timestamp)
    boot_string = (str(bt.year) + "/" + str(bt.month) + "/" + str(bt.day) + " " + str(bt.hour) + ":" + str(bt.minute)
                   + ":" + str(bt.second))
    update_lcd(display_speed, "Boot Time:", boot_string)


def get_cpu_stats():
    # CAUTION: Hardcoded some CPU stats to 1 second updates

    update_lcd(display_speed, "Physical cores:", str(psutil.cpu_count(logical=False)))
    update_lcd(display_speed, "Logical cores:", str(psutil.cpu_count(logical=True)))
    update_lcd(display_speed, "CPU Max Freq:", str(round(psutil.cpu_freq().max, 2)) + "MHz")
    update_lcd(display_speed, "CPU Min Freq:", str(round(psutil.cpu_freq().min, 2)) + "MHz")

    for j in range(display_speed):
        update_lcd(1, "Current Freq:", str(round(psutil.cpu_freq().current, 2)) + "MHz")

    # Show usage per core
    for i, percentage in enumerate(psutil.cpu_percent(percpu=True, interval=1)):
        for j in range(display_speed):
            update_lcd(1, "CPU Load:", "Core " + str(i) + ": " + str(percentage) + "%")

    for j in range(display_speed):
        update_lcd(1, "CPU Load:", "Total: " + str(psutil.cpu_percent()) + "%")


def get_memory():
    update_lcd(display_speed, "Total RAM:", str(int(psutil.virtual_memory().total / 1048576)) + "MiB")
    update_lcd(display_speed, "Used RAM:", str(int(psutil.virtual_memory().used / 1048576)) + "MiB")
    update_lcd(display_speed, "Available RAM:", str(int(psutil.virtual_memory().available / 1048576)) + "MiB")
    update_lcd(display_speed, "RAM Usage:", str(psutil.virtual_memory().percent) + "%")

    update_lcd(display_speed, "Total Swap:", str(int(psutil.swap_memory().total / 1048576)) + "MiB")
    update_lcd(display_speed, "Used Swap:", str(int(psutil.swap_memory().used / 1048576)) + "MiB")
    update_lcd(display_speed, "Available Swap:", str(int(psutil.swap_memory().free / 1048576)) + "MiB")
    update_lcd(display_speed, "Swap Usage:", str(psutil.swap_memory().percent) + "%")


def get_network():
    interfaces = psutil.net_if_addrs()
    for name, addresses in interfaces.items():
        for address in addresses:
            update_lcd(display_speed, name, "Address: " + str(address.address))
            update_lcd(display_speed, name, "Netmask: " + str(address.netmask))
            update_lcd(display_speed, name, "Broadcast: " + str(address.broadcast))

    for i in range(display_speed):
        update_lcd(1, "Bytes Sent:", str(psutil.net_io_counters().bytes_sent))
    for i in range(display_speed):
        update_lcd(1, "Bytes Received:", str(psutil.net_io_counters().bytes_recv))

    update_lcd(display_speed, "Errors Out:", str(psutil.net_io_counters().errout))
    update_lcd(display_speed, "Errors In:", str(psutil.net_io_counters().errin))


def get_gpu_stats():
    gpus = GPUtil.getGPUs()
    for gpu in gpus:
        for i in range(display_speed):
            update_lcd(1, gpu.name, "Load: " + f"{gpu.load*100}%")

        for i in range(display_speed):
            update_lcd(1, gpu.name, "Free Mem: " + f"{gpu.memoryFree}MB")

        for i in range(display_speed):
            update_lcd(1, gpu.name, "Used Mem: " + f"{gpu.memoryUsed}MB")

        update_lcd(display_speed, gpu.name, "Total Mem: " + f"{gpu.memoryTotal}MB")

        for i in range(display_speed):
            update_lcd(1, gpu.name, "Temperature: " + f"{gpu.temperature}C")


if __name__ == "__main__":

    print("Looping System Info")

    while True:
        get_system_uname()
        get_boot_time()
        get_cpu_stats()
        get_memory()
        get_network()
        get_gpu_stats()
