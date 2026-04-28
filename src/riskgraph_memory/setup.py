from setuptools import setup

package_name = 'riskgraph_memory'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusufdxb',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='Persistent risk memory ROS 2 node (SQLite-backed)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'riskgraph_memory_node = riskgraph_memory.memory_node:main',
            'riskgraph_safety_adapter = riskgraph_memory.adapters.safety_adapter:main',
            'riskgraph_helix_adapter = riskgraph_memory.adapters.helix_adapter:main',
            'riskgraph_tactile_adapter = riskgraph_memory.adapters.tactile_adapter:main',
        ],
    },
)
