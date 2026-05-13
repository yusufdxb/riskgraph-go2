from setuptools import setup
import os
from glob import glob

package_name = 'riskgraph_bringup'

setup(
    name=package_name,
    version='0.1.1',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'segment_seeds'),
            glob('config/segment_seeds/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusufdxb',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='Launch files and configs for RiskGraph-Go2',
    license='MIT',
)
