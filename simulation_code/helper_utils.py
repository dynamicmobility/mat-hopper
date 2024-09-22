import math
import xml.etree.ElementTree as ET
import os
import contextlib

def modulus_to_stiffness(modulus, mat_radius = 0.025, spring_length=0.01):
    inertia=math.pi * mat_radius * 4
    return modulus * 3 * inertia / (spring_length ** 3)

def density_conversion(density):
    return density * 1000

def parse_file(filename, output_filename, density, stiffness, color1, color2, color3):
    tree = ET.parse(filename)
    root = tree.getroot()
    if isinstance(density, str):
        density = [density] * 3
    else:
        density = list(map(lambda x: str(x), density))
    if isinstance(stiffness, str):
        stiffness = [str(stiffness)] * 3
    else: 
        stiffness = list(map(lambda x: str(x), stiffness))
    if not isinstance(color1, str):
        color1 = f"{color1[0]} {color1[1]} {color1[2]} 1"
    if not isinstance(color2, str):
        color2 = f"{color2[0]} {color2[1]} {color2[2]} 1"
    if not isinstance(color3, str):
        color3 = f"{color3[0]} {color3[1]} {color3[2]} 1"
        
    for element in root.iter("geom"):
        if element.get('name') == 'mat1_1' or element.get('name') == 'mat1_2':
            element.set('density', density[0])
            element.set('rgba', color1)
        if element.get('name') == 'mat2_1' or element.get('name') == 'mat2_2':
            element.set('density', density[1])
            element.set('rgba', color2)
        if element.get('name') == 'mat3_1' or element.get('name') == 'mat3_2':
            element.set('density', density[2])
            element.set('rgba', color3)
    for element in root.iter("joint"):
        if element.get('name') == 'mat_spring1':
            element.set('stiffness', stiffness[0])
        if element.get('name') == 'mat_spring2':
            element.set('stiffness', stiffness[1])
        if element.get('name') == 'mat_spring3':
            element.set('stiffness', stiffness[2])
            break
    tree.write(output_filename, encoding='utf-8', xml_declaration=True)
    return output_filename

def parse_file_dt(filename, output_filename, dt):
    tree = ET.parse(filename)
    root = tree.getroot()
    if not isinstance(dt, str):
        dt = str(dt)
    for element in root.iter("option"):
        element.set('timestep', dt)
        break
    tree.write(output_filename, encoding='utf-8', xml_declaration=True)
    return output_filename

def make_basic_materials():
    base_filename = os.path.join("simulation_code","base-material.xml")
    material_folder = os.path.join("simulation_code","created_models")
    if not os.path.exists(material_folder):
        os.makedirs(material_folder)
    parse_file(base_filename, os.path.join(material_folder,"pvc.xml"), "1390", "3.77E6", "0.729411765 0.635294118 0.450980392 1", "0.729411765 0.635294118 0.450980392 1", "0.729411765 0.635294118 0.450980392 1")
    parse_file(base_filename, os.path.join(material_folder,"aluminum.xml"), "2700", "6.6E7", "0.611764706 0.615686275 0.709803922 1", "0.611764706 0.615686275 0.709803922 1", "0.611764706 0.615686275 0.709803922 1")
    parse_file(base_filename, os.path.join(material_folder,"titanium.xml"), "4430", "1.13E8", "0.498039216 0.501960784 0.674509804 1", "0.498039216 0.501960784 0.674509804 1", "0.498039216 0.501960784 0.674509804 1")
    parse_file(base_filename, os.path.join(material_folder,"stainlesssteel.xml"), "8000", "1.82E8", "0.411764706 0.419607843 0.658823529 1", "0.411764706 0.419607843 0.658823529 1", "0.411764706 0.419607843 0.658823529 1")
    # parse_file(base_filename, os.path.join(material_folder,"alum-oxide.xml"), "3900", "3.2E8")