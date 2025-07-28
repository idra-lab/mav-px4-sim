import random

import xml.etree.ElementTree as ET

worlds_folder = "worlds/"

# Path to your original and new world files
input_file = worlds_folder +  "easy_forest.sdf"
output_file = worlds_folder + "medium_forest.sdf"

# The model URI to include
model_uri = "model://models/pine_tree"

# Parse the original SDF file
tree = ET.parse(input_file)
root = tree.getroot()

# SDF tags can be under <sdf> or <world>
world = root.find("world")
if world is None:
    raise RuntimeError("Could not find <world> tag in SDF.")

# Collect <model> tags with numeric names
models_to_replace = [model for model in world.findall("model") if model.attrib.get("name", "").isdigit()]

# Replace them with <include> blocks
for model in models_to_replace:
    model_id = model.attrib["name"]

    # Get the pose
    pose_elem = model.find("pose")
    pose_text = pose_elem.text.strip() if pose_elem is not None else "0 0 0 0 0 0"

    # Set the third element (z position) of the pose to zero
    pose_values = pose_text.split()
    if len(pose_values) >= 3:
        pose_values[2] = "0"
    pose_text = " ".join(pose_values)

    # Create new <include> element
    include = ET.Element("include")
    
    uri = ET.SubElement(include, "uri")
    uri.text = model_uri
    
    name = ET.SubElement(include, "name")
    name.text = f"pine_{model_id}"

    pose = ET.SubElement(include, "pose")
    pose.text = pose_text

    

    # Insert <include> before the old model, then remove the model
    world.insert(list(world).index(model), include)
    world.remove(model)

# Save the modified file
tree.write(output_file, encoding="utf-8", xml_declaration=True)

print(f"Successfully converted {len(models_to_replace)} models and wrote to {output_file}")
