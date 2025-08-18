import psutil
import os

# Total number of CPU cores
cpu_cores = psutil.cpu_count(logical=True)      # logical = includes hyperthreading
cpu_physical = psutil.cpu_count(logical=False)  # only physical cores

# Total RAM in GB
total_ram = psutil.virtual_memory().total / (1024 ** 3)

print(f"Physical cores: {cpu_physical}")
print(f"Logical cores: {cpu_cores}")
print(f"Total RAM: {total_ram:.2f} GB")

# Optional: show system load average (Linux/Unix)
if hasattr(os, "getloadavg"):
    load1, load5, load15 = os.getloadavg()
    print(f"System load (1m, 5m, 15m): {load1}, {load5}, {load15}")