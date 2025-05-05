from setuptools import find_packages, setup

package_name = 'catch_turtle'

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
    maintainer='ubuntum1',
    maintainer_email='abhishek.kumar.ubuntu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_controller = catch_turtle.turtle_controller:main",
            "turtle_spawner = catch_turtle.turtle_spawner:main"
        ],
    },
)
