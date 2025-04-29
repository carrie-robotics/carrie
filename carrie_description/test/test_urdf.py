import os
import xacro
import pytest
from ament_index_python.packages import get_package_share_directory

def test_xacro_compiles():
    xacro_path = os.path.join(get_package_share_directory("carrie_description"), 'urdf', 'carrie.urdf.xacro')
    try:
        doc = xacro.process_file(xacro_path)
        xml = doc.toxml()
    except Exception as e:
        pytest.fail(f"xacro failed to parse: {e}")

