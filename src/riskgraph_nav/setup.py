from setuptools import setup

package_name = 'riskgraph_nav'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusufdxb',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='Live GO2 navigation integration for RiskGraph: localization anchor, '
                'preflight, trial runner, evidence bundle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'riskgraph_localization = riskgraph_nav.localization_node:main',
            'riskgraph_preflight = riskgraph_nav.preflight:main',
            'riskgraph_live_trial = riskgraph_nav.live_trial:main',
            'riskgraph_status = riskgraph_nav.status_cli:main',
            'riskgraph_generate_course_map = riskgraph_nav.course_map_cli:main',
            'riskgraph_replay_check = riskgraph_nav.replay_check:main',
            'riskgraph_rehearsal_go2 = riskgraph_nav.rehearsal_go2:main',
            'riskgraph_sport_sink = riskgraph_nav.sport_sink:main',
            'riskgraph_sink_stage = riskgraph_nav.sink_stage:main',
        ],
    },
)
