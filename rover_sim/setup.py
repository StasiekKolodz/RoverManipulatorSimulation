from setuptools import setup

package_name = 'rover_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rover_urdf', ['rover_urdf/rover.urdf.xacro']),
        ('share/' + package_name + '/rover_urdf', ['rover_urdf/rover_gazebo.xacro']),
        ('share/' + package_name + '/launch', ['launch/rover_sim.launch.py']),
        ('share/' + package_name + '/params', ['params/rover.yaml']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stas',
    maintainer_email='stas.kolodziejczyk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
