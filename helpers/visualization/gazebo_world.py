import os
import xml.etree.ElementTree as ET


def update_world(drones,markers, world_path):
    world_file_path = os.path.expanduser(world_path)

    # Load the existing SDF file
    tree = ET.parse(world_file_path)
    root = tree.getroot()

    # Find the <world> element
    world_elem = root.find("world")

    # # Remove old waypoints and drones
    # for model in world_elem.findall("model"):
    #     model_name = model.attrib.get("name", "")
    #     if "green_waypoint" in model_name or "red_waypoint" in model_name or "drone" in model_name: #
    #         world_elem.remove(model)

    # Add makers
    for name,marker_set in markers.items():
        positions=marker_set.pop('pos')
        for i, (x, y, z) in enumerate(positions):
            waypoint_elem = generate_waypoint_element(f"{name}_{i}", x, y, z,**marker_set)
            world_elem.append(waypoint_elem)

    # Add drones
    for i, (x, y, z, roll, pitch, yaw) in enumerate(drones):
        drone_elem = generate_drone_element(f"drone{i+1}", x, y, z, roll, pitch, yaw)
        world_elem.append(drone_elem)

    # Save the modified world file
    updated_world_path = world_path[:-6] + "_updated.world"
    updated_world_path = os.path.expanduser(updated_world_path)
    tree.write(updated_world_path)

    return updated_world_path







def generate_drone_element(name, x, y, z, roll, pitch, yaw):
    """Creates an XML element for a drone model."""
    model = ET.Element("model", name=name)
    
    pose = ET.SubElement(model, "pose")
    pose.text = f"{x} {y} {z} {roll} {pitch} {yaw}"

    include = ET.SubElement(model, "include")
    uri = ET.SubElement(include, "uri")
    uri.text = f"model://{name}"

    return model


import xml.etree.ElementTree as ET


def generate_waypoint_element(name, x, y, z, color="green", radius=0.2, alpha=0.05):
    """Creates a fully defined XML element for a waypoint model with configurable color, radius, and transparency."""
    import xml.etree.ElementTree as ET

    # Define color mappings
    color_map = {
        "green": "0.306 0.604 0.024 1",
        "red": "0.8 0.0 0.0 1",
        "yellow": "1.0 1.0 0.0 1",
        "orange": "1.0 0.5 0.0 1",
        "blue": "0.0 0.0 1.0 1"
    }

    diffuse_color = color_map.get(color.lower(), color_map["green"])

    model = ET.Element("model", name=name)

    pose = ET.SubElement(model, "pose")
    pose.text = f"{x} {y} {z} 0 0 0"

    link = ET.SubElement(model, "link", name="link")

    # Inertial properties
    inertial = ET.SubElement(link, "inertial")
    for tag, value in {
        "mass": "1",
        "ixx": "0.1", "ixy": "0", "ixz": "0",
        "iyy": "0.1", "iyz": "0", "izz": "0.1"
    }.items():
        if tag == "mass":
            ET.SubElement(inertial, tag).text = value
        else:
            ET.SubElement(ET.SubElement(inertial, "inertia"), tag).text = value
    ET.SubElement(inertial, "pose").text = "0 0 0 0 -0 0"

    # Physics
    for tag in ["self_collide", "enable_wind", "kinematic", "gravity"]:
        ET.SubElement(link, tag).text = "0"
    ET.SubElement(link, "pose").text = "0 0 0 0 -0 0"

    # Visual
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = str(radius)

    material = ET.SubElement(visual, "material")
    script = ET.SubElement(material, "script")
    ET.SubElement(script, "name").text = "Gazebo/Grey"
    ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"

    shader = ET.SubElement(material, "shader", type="pixel")
    ET.SubElement(shader, "normal_map").text = "__default__"

    ET.SubElement(material, "ambient").text = "0.3 0.3 0.3 1"
    ET.SubElement(material, "diffuse").text = diffuse_color
    ET.SubElement(material, "specular").text = "0.01 0.01 0.01 1"
    ET.SubElement(material, "emissive").text = "0 0 0 1"

    ET.SubElement(visual, "pose").text = "0 0 0 0 -0 0"
    ET.SubElement(visual, "transparency").text = str(alpha)
    ET.SubElement(visual, "cast_shadows").text = "1"

    # Static properties
    ET.SubElement(model, "static").text = "0"
    ET.SubElement(model, "allow_auto_disable").text = "1"

    return model


















# def generate_waypoint_element(name, x, y, z, color="green"):
#     """Creates a fully defined XML element for a waypoint model with configurable color."""
    
#     # Define color mappings
#     color_map = {
#         "green": "0.306 0.604 0.024 1",  # Green
#         "red": "0.8 0.0 0.0 1" , # Red
#         "yellow": "1.0 1.0 0.0 1" , # Yellow
#         "orange": "1.0 0.5 0.0 1" ,# Orange
#         "blue": "0.0 0.0 1.0 1"  # Blue
#     }

#     # Default to green if the color is not recognized
#     diffuse_color = color_map.get(color.lower(), color_map["green"])

#     model = ET.Element("model", name=name)

#     pose = ET.SubElement(model, "pose")
#     pose.text = f"{x} {y} {z} 0 0 0"

#     link = ET.SubElement(model, "link", name="link")

#     # Inertial properties
#     inertial = ET.SubElement(link, "inertial")
#     mass = ET.SubElement(inertial, "mass")
#     mass.text = "1"
#     inertia = ET.SubElement(inertial, "inertia")
#     ixx = ET.SubElement(inertia, "ixx")
#     ixx.text = "0.1"
#     ixy = ET.SubElement(inertia, "ixy")
#     ixy.text = "0"
#     ixz = ET.SubElement(inertia, "ixz")
#     ixz.text = "0"
#     iyy = ET.SubElement(inertia, "iyy")
#     iyy.text = "0.1"
#     iyz = ET.SubElement(inertia, "iyz")
#     iyz.text = "0"
#     izz = ET.SubElement(inertia, "izz")
#     izz.text = "0.1"

#     inertial_pose = ET.SubElement(inertial, "pose")
#     inertial_pose.text = "0 0 0 0 -0 0"

#     # Physics settings
#     self_collide = ET.SubElement(link, "self_collide")
#     self_collide.text = "0"
    
#     enable_wind = ET.SubElement(link, "enable_wind")
#     enable_wind.text = "0"
    
#     kinematic = ET.SubElement(link, "kinematic")
#     kinematic.text = "0"
    
#     link_pose = ET.SubElement(link, "pose")
#     link_pose.text = "0 0 0 0 -0 0"
    
#     gravity = ET.SubElement(link, "gravity")
#     gravity.text = "0"

#     # Visual properties
#     visual = ET.SubElement(link, "visual", name="visual")
#     geometry = ET.SubElement(visual, "geometry")
#     sphere = ET.SubElement(geometry, "sphere")
#     radius = ET.SubElement(sphere, "radius")
#     radius.text = "0.2"

#     material = ET.SubElement(visual, "material")
#     script = ET.SubElement(material, "script")
#     name_script = ET.SubElement(script, "name")
#     name_script.text = "Gazebo/Grey"
#     uri = ET.SubElement(script, "uri")
#     uri.text = "file://media/materials/scripts/gazebo.material"

#     shader = ET.SubElement(material, "shader", type="pixel")
#     normal_map = ET.SubElement(shader, "normal_map")
#     normal_map.text = "__default__"

#     ambient = ET.SubElement(material, "ambient")
#     ambient.text = "0.3 0.3 0.3 1"

#     diffuse = ET.SubElement(material, "diffuse")
#     diffuse.text = diffuse_color  # Set color dynamically

#     specular = ET.SubElement(material, "specular")
#     specular.text = "0.01 0.01 0.01 1"

#     emissive = ET.SubElement(material, "emissive")
#     emissive.text = "0 0 0 1"

#     visual_pose = ET.SubElement(visual, "pose")
#     visual_pose.text = "0 0 0 0 -0 0"

#     transparency = ET.SubElement(visual, "transparency")
#     transparency.text = "0.05"

#     cast_shadows = ET.SubElement(visual, "cast_shadows")
#     cast_shadows.text = "1"

#     # Static properties
#     static = ET.SubElement(model, "static")
#     static.text = "0"

#     allow_auto_disable = ET.SubElement(model, "allow_auto_disable")
#     allow_auto_disable.text = "1"

#     return model





