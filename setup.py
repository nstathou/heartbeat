from setuptools import setup

package_name = 'heartbeat'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niksta',
    maintainer_email='nstathou@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bs_talker = heartbeat.bs_talker:main',
            'bs_listener = heartbeat.bs_listener:main',
            'agent_listener = heartbeat.agent_listener:main',
            'agent_talker = heartbeat.agent_talker:main'
        ],
    },
)
