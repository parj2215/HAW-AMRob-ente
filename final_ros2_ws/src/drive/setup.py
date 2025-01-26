from setuptools import setup

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drive.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Description of the drive package',
    license='License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_fsm = drive.fsm_drive:main',
        ],
    },
)
