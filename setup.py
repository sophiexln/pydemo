from setuptools import setup
import os
from glob import glob

package_name = 'pydemo'

setup(
    name=package_name,
    version='0.0.0',
    packages=['pydemo', 'pydemo.nodes', 'pydemo.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('pydemo', 'config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayeon',
    maintainer_email='shinhy9@naver.com',
    description='py_trees + Navigation2 + Image Capture',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_image_service = pydemo.nodes.capture_image_service:main',
            'py_trees_main = pydemo.py_trees_main:main',
            'reference = pydemo.scripts.reference:main',
            'capture = pydemo.scripts.capture:main',
            'reference_service = pydemo.nodes.reference_service:main',
            'reference_node = pydemo.nodes.reference_node:main',
        ],
    },
)