from pathlib import Path
import json,xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
p=Path(get_package_share_directory('balance_sim'))
models={}
for contact in ['sphere','cylinder']:
 root=ET.fromstring(xacro.process_file(str(p/'description/balance.urdf.xacro'),mappings={'wheel_contact':contact}).toxml())
 models[contact]={'links':len(root.findall('link')),'joints':len(root.findall('joint')),
                 'left_collision':ET.tostring(root.find("link[@name='left_wheel']/collision/geometry"),encoding='unicode'),
                 'left_inertia':root.find("link[@name='left_wheel']/inertial/inertia").attrib}
assert models['sphere']['left_inertia']==models['cylinder']['left_inertia']
Path(__file__).with_name('contact_expansion.json').write_text(json.dumps(models,indent=2))
print('Two collision options: links/joints/inertia unchanged; geometry changed.')
