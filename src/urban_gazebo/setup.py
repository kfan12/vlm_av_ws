import os
from glob import glob

from setuptools import setup

package_name = 'urban_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
    ] + [
        # sign models: model.config/model.sdf per model dir
        (os.path.join('share', package_name, 'models',
                      os.path.basename(d)), glob(os.path.join(d, 'model.*')))
        for d in glob('models/*')
    ] + [
        # sign meshes: obj + mtl + png per model dir (from-empty deviation:
        # assets are generated locally, not borrowed from a v1 package)
        (os.path.join('share', package_name, 'models',
                      os.path.basename(d), 'meshes'),
         glob(os.path.join(d, 'meshes', '*')))
        for d in glob('models/*')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kefan',
    maintainer_email='tju.fanke@gmail.com',
    description='V2 urban sim tooling',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
