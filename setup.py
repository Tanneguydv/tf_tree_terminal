from setuptools import find_packages, setup

package_name = 'tf_tree_terminal'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('bin', ['bin/tf-tree']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanneguy de Villemagne',
    maintainer_email='tanneguydv@gmail.com',
    description='A lightweight ROS 2 utility to visualize the Coordinate Transform (TF) tree directly in the terminal with a folder-style structure.',
    license='MIT License',
    tests_require=['pytest'],
    scripts=['bin/tf-tree'],
    entry_points={
        'console_scripts': [
            'show = tf_tree_terminal.tree_node:main'
        ],
    },
)
