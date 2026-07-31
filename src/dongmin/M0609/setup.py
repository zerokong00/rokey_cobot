from setuptools import find_packages, setup

package_name = 'M0609'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='loik1235@gmail.com',
    description='Isaac Sim M0609 wrist camera color detection node (blue=1 / green=2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm0609_color_detector = M0609.m0609_color_detector:main',
        ],
    },
)
