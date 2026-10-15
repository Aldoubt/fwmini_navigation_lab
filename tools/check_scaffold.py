"""Dependency-free static checks; not a ROS build."""
import ast
import pathlib
import xml.etree.ElementTree as ET
root = pathlib.Path(__file__).resolve().parents[1]
for path in root.glob('src/*/package.xml'):
    package = ET.parse(path).getroot()
    assert package.findtext('name') == path.parent.name
    assert (path.parent / 'CMakeLists.txt').exists()
for path in root.glob('src/**/*.py'):
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
robot = ET.parse(root / 'src/fwmini_description/urdf/fwmini.urdf.xacro').getroot()
links = {link.attrib['name'] for link in robot.findall('link')}
children = []
for joint in robot.findall('joint'):
    assert joint.find('parent').attrib['link'] in links
    child = joint.find('child').attrib['link']
    assert child in links and child not in children
    children.append(child)
assert links - set(children) == {'base_link'}
print('PASS: package manifests, launch syntax and static TF topology')
