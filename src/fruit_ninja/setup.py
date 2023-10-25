from setuptools import find_packages, setup

package_name = 'fruit_ninja'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tan',
    maintainer_email='tan@metu.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fruit_game = fruit_ninja.fruit_game:main',
            'finger_camera_handler = fruit_ninja.finger:main'
        ],
    },
)
