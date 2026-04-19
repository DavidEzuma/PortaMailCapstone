from setuptools import setup

package_name = 'lcd_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lcd_bridge']),
        ('share/lcd_bridge', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 bridge skeleton for PortaMail LCD Flask server.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lcd_bridge = lcd_bridge.node:main',
        ],
    },
)
