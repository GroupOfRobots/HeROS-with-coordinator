from setuptools import setup

package_name = 'heros_test_scenario'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='HeROS project - cubes transport scenario',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cubes_transport = heros_test_scenario.cubes_transport:main",
        ],
    },
)
