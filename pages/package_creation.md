# Simple python publisher subscriber
Example package with instructions on how to easily set up a couple of python publisher and subcriber nodes.

## Set up
Create a python package
'''
ros2 pkg create --build-type ament_python <package_name>
# ros2 pkg create --build-type ament_python simple_python_publish_subscribe
'''

## Add executables

Open the setup.py file and replace the sniped accordingly:
Remeber that your scripts have to be placed inside the folder with the same name as your package.
'''
        import os
        from glob import glob
        from setuptools import setup
        ...
        data_files=[
                ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ],
        ...
        entry_points={
                'console_scripts': [
                        '<node1> = <package_name>.<file_name1>:<main_function1>',
                        '<node2> = <package_name>.<file_name2>:<main_function2>',
                ],
        },
#   entry_points={
#       'console_scripts': [
#           'publisher = simple_python_publish_subscribe.publisher:main',
#           'subscriber = simple_python_publish_subscribe.subscriber:main',
#       ],
#   },
'''
