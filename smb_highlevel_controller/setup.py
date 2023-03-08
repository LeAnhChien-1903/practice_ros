from setuptools import setup

package_name = 'smb_highlevel_controller'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/smb_highlevel_controller.launch.py']))
data_files.append(('share/' + package_name + '/config', ['config/params.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leanhchien',
    maintainer_email='leanhchien1903@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_subscriber = smb_highlevel_controller.minimal_subscriber:main',
            'p_controller = smb_highlevel_controller.p_controller:main'
        ],
    },
)
