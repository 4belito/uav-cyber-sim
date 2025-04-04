
import os
import subprocess
from simulators.sim import Simulator,SimName
from helpers.change_coordinates import find_spawns

from typing import List,Tuple
from config import QGC_INI_PATH,QGC_PATH


class QGC(Simulator):
    def __init__(self, offsets: List[Tuple],origin:Tuple):
        super().__init__(name=SimName.QGROUND,offsets=offsets) 
        self.add_origin(origin)


    def add_origin(self, origin):
        self.add_info('origin', origin)
        self.info['spawns'] = find_spawns(origin, self.offsets)

    def add_vehicle_cmd_fn(self,i):
        spawn_str=','.join(map(str, self.info['spawns'][i]))
        return f" --custom-location={spawn_str}"
    
    def launch(self):
        delete_all_qgc_links()
        add_qgc_links(n=self.n_uavs)
        sim_cmd = [os.path.expanduser(QGC_PATH)]
        subprocess.Popen(
            sim_cmd,
            stdout=subprocess.DEVNULL,  # Suppress standard output
            stderr=subprocess.DEVNULL,  # Suppress error output
            shell=False  # Ensure safety when passing arguments
            )
    

def add_qgc_links(n:int=1, start_port:int=5763, step:int=10):
    """
    Adds n new TCP link configurations to QGroundControl's .ini file.

    Args:
        n (int): Number of new links to add.
        start_port (int): Starting port number.
        step (int): Step between ports.
        ini_path (str): Optional path to the .ini file. If None, uses default.
    """
    # Read the original file
    with open(QGC_INI_PATH, 'r') as file:
        lines = file.readlines()

    # Find the [LinkConfigurations] section
    start_idx = None
    for idx, line in enumerate(lines):
        if line.strip() == '[LinkConfigurations]':
            start_idx = idx
            break

    if start_idx is None:
        raise Exception("Could not find the [LinkConfigurations] section")

    # Find the 'count=' line and get current count
    count_line_idx = next(i for i in range(start_idx, len(lines)) if lines[i].startswith('count='))
    current_count = int(lines[count_line_idx].split('=')[1])

    # Create new connection blocks
    new_lines = []
    for i in range(n):
        idx = current_count + i
        port = start_port + step * i
        new_lines.extend([
            f'Link{idx}\\auto=true\n',
            f'Link{idx}\\high_latency=false\n',
            f'Link{idx}\\host=127.0.0.1\n',
            f'Link{idx}\\name=drone{idx + 1}\n',
            f'Link{idx}\\port={port}\n',
            f'Link{idx}\\type=2\n'
        ])

    # Insert new lines before 'count=' and update the count
    lines[count_line_idx:count_line_idx] = new_lines
    lines[count_line_idx + len(new_lines)] = f'count={current_count + n}\n'

    # Save the updated file
    with open(QGC_INI_PATH, 'w') as file:
        file.writelines(lines)

    print(f"✅ {n} new connection(s) added starting from port {start_port}. Restart QGroundControl to see them.")



def delete_all_qgc_links():
    with open(QGC_INI_PATH, 'r') as f:
        lines = f.readlines()

    inside_links = False
    new_lines = []

    for line in lines:
        if line.strip() == "[LinkConfigurations]":
            inside_links = True
            new_lines.append(line)
            new_lines.append("count=0\n")  # reset count
            continue

        if inside_links:
            if line.startswith("Link") or line.startswith("count="):
                continue  # skip all LinkX and count lines
            elif line.startswith("["):  # next section begins
                inside_links = False

        new_lines.append(line)

    with open(QGC_INI_PATH, 'w') as f:
        f.writelines(new_lines)

    print("🧼 All QGroundControl links deleted (count=0).")