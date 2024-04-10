from setuptools import find_packages, setup

package_name = 'cv_algorithm'

setup(
    name=package_name,
    version='0.0.0',  # Update the version as needed
    packages=find_packages(exclude=['test']),  # Automatically find and include all packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python==4.6.0.66',  # Specified version of opencv-python
	'ultralytics',
        # Additional dependencies like torch and torchvision should be specified here,
        # but since YOLOv8 might have specific requirements, you might handle those separately
    ],
    zip_safe=True,
    maintainer='SUMS',
    maintainer_email='henrik-21@hotmail.com',
    description='A ROS 2 package for computer vision algorithms using PyTorch, including YOLOv8.',
    license='MIT',  # Update with your actual license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'computer_vision=cv_algorithm.computer_vision:main',
	],
    },
)
