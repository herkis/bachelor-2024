from setuptools import setup

package_name = 'modem_communication'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'unetpy'],
    zip_safe=True,
    maintainer='Peder',
    maintainer_email='pederbsa@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'modem_data_handler = OLD_modem_communication.modem_data_handler_node_main:main'
            'modem_data_handler = modem_communication.modem_data_handler_node_main:main',
            'modem_data_sender = modem_communication.modem_data_sender_node_main:main'
        ],
    },
)
