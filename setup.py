from setuptools import find_packages, setup

package_name = 'vs_search'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adiprash',
    maintainer_email='aditya.m.prashanth@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'datum_node = vs_search.datum_node:main',
            'boat_node = vs_search.boat_node:main',
            'boat_manager = vs_search.boat_manager:main',
        ],
    },
)
