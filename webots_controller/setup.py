from setuptools import setup

package_name = 'webots_controller'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/webots_controller.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/pioneer3at_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/pioneer3at_webots.urdf']))
data_files.append(('share/' + package_name + '/models', ['models/pioneer3at.urdf']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/urdf_config.rviz']))
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files= data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leanhchien',
    maintainer_email='leanhchien1903@gmail.com',
    description='webots_controller with pioneer3-AT platform ',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = webots_controller.controller:main'
        ],
    },
)
