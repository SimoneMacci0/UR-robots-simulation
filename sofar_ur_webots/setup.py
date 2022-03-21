from setuptools import setup

package_name = 'sofar_ur_webots'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/simulation.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robots_controller.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/universal_robots.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/webots_ur5e_description.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/ros2_control_config.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_controller = sofar_ur_webots.ur_controller:main',
        ],
    },
)
