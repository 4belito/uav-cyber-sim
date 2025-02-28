import os
import xml.etree.ElementTree as ET

def generate_drone_element(name, x, y, z, roll, pitch, yaw):
    """Creates an XML element for a drone model."""
    model = ET.Element("model", name=name)
    
    pose = ET.SubElement(model, "pose")
    pose.text = f"{x} {y} {z} {roll} {pitch} {yaw}"

    include = ET.SubElement(model, "include")
    uri = ET.SubElement(include, "uri")
    uri.text = f"model://{name}"

    return model

def update_world(drones,world_path):
    world_file_path = os.path.expanduser(world_path)

    # Load the existing SDF file
    tree = ET.parse(world_file_path)
    root = tree.getroot()

    # Find the <world> element
    world_elem = root.find("world")

    # Remove old drone models (if they exist)
    for model in world_elem.findall("model"):
        if "drone" in model.attrib.get("name", ""):
            world_elem.remove(model)

    # Add new drones dynamically
    for drone in drones:
        drone_elem = generate_drone_element(*drone)
        world_elem.append(drone_elem)

    # Save the modified world file
    updated_world_path=world_path[:-6]+"_updated.world"
    updated_world_path = os.path.expanduser(updated_world_path)
    tree.write(updated_world_path)
    return updated_world_path