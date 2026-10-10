"""Small MicroPython test: create a CSV file and write one row."""

import time

filename = "log_test.csv"

with open(filename, "w") as log_file:
    log_file.write("time_ms,event,message\n")
    log_file.write(str(time.ticks_ms()) + ",TEST,ESP32 file write OK\n")

print("Wrote test row to " + filename)
