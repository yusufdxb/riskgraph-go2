from setuptools import setup
import os
from glob import glob

package_name = 'riskgraph_demo'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'fixtures'),
            glob('fixtures/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusufdxb',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='Synthetic data + offline demo orchestrator for RiskGraph-Go2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'riskgraph_synthetic_publisher = riskgraph_demo.synthetic_publisher:main',
            'riskgraph_offline_demo = riskgraph_demo.offline_demo:main',
        ],
    },
)
