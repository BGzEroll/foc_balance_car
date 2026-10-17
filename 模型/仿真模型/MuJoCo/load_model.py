"""Read with pathlib so Windows paths with Chinese characters work too."""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
def load_model():
    folder=Path(__file__).resolve().parent
    xml=ET.fromstring((folder/'balance_car.xml').read_text(encoding='utf-8'))
    xml.find('compiler').set('meshdir','')
    assets={p.name:p.read_bytes() for p in (folder/'meshes').glob('*.stl')}
    return mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'),assets=assets)
