from setuptools import setup

package_name = 'riskgraph_planner'

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
    description='Route scoring service for RiskGraph-Go2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'riskgraph_planner_node = riskgraph_planner.planner_node:main',
        ],
    },
)
