from setuptools import setup

package_name = 'omx_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Forward kinematics node for the OpenManipulator-X (RBE 500)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # "ros2 run omx_kinematics forward_kinematics" runs main() in forward_kinematics.py
            'forward_kinematics = omx_kinematics.forward_kinematics:main',
        ],
    },
)
