import xml.etree.ElementTree as ET
def parse_file(filename, output_filename, density, stiffness):
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

    for element in root.iter("geom"):
        if element.get('name') == 'mat1_1' or element.get('name') == 'mat1_2':
            element.set('density', density[0])
        if element.get('name') == 'mat2_1' or element.get('name') == 'mat2_2':
            element.set('density', density[1])
        if element.get('name') == 'mat3_1' or element.get('name') == 'mat3_2':
            element.set('density', density[2])
    for element in root.iter("joint"):
        print(element)
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

if __name__ == "__main__":
    base_filename = "base.xml"
    # parse_file(base_filename, "aluminum.xml", ["2700","2700","2700"], ["6.6E7","6.6E7","6.6E7"])
    parse_file(base_filename, "pvc.xml", "1390", "3.77E6")
    parse_file(base_filename, "aluminum.xml", "2700", "6.6E7")
    parse_file(base_filename, "titanium.xml", "4430", "1.13E8")
    parse_file(base_filename, "stainlesssteel.xml", "8000", "1.82E8")
    parse_file(base_filename, "alum-oxide.xml", "3900", "3.2E8")