from setuptools import setup

package_name = 'dobot_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dobot_viz.launch.py']),
        ('share/' + package_name + '/launch', ['launch/dobot_sim.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/dobot_viz.urdf.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/dobot_gazebo.xacro']),

        ('share/' + package_name + '/rviz', ['rviz/dobot_viz.rviz']),
        ('share/' + package_name + '/params', ['params/robot_size.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siwy',
    maintainer_email='01168879@pw.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        ["ForwardKin = dobot_viz.ForwardKin:main",
        "robot_control = dobot_viz.robot_control:main",
        "move_to_point = dobot_viz.move_to_point:main",
        "marker_publisher=dobot_viz.marker_publisher:main",
        "marker_broker=dobot_viz.marker_broker:main"
        ],
    },
)
