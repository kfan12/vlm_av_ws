import os
from glob import glob

from setuptools import setup

package_name = 'vlm_planner_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kefan',
    maintainer_email='tju.fanke@gmail.com',
    description='VLM sign chain (venv nodes).',
    license='MIT',
    # NO console_scripts on purpose: `ros2 run` would execute them with the
    # system-python shebang (no torch). Venv nodes are launched as modules:
    #     python3 -m vlm_planner_py.vlm_sign_node
    entry_points={'console_scripts': []},
)