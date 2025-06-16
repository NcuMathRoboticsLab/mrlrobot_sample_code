from setuptools import setup

package_name = 'teleop'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrl',
    maintainer_email='mathroboticslab@gmail.com',
    description='mrlrobot teleop',
    license='No License declaration',
    entry_points={
        'console_scripts': [
            'teleop_key = teleop.teleop_key:main',
        ],
    },
)
