from setuptools import find_packages, setup

package_name = 'image_client'

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
    maintainer='SUMS',
    maintainer_email='henrik-21@hotmail.com',
    description='Client for sending images via modem',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['bilde_klient=image_client.bilde_klient:main',
        ],
    },
)
