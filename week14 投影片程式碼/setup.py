from setuptools import setup

setup(
    name='tello',
    version='0.1.0',
    packages=['tello'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tello']),
        ('share/tello', ['package.xml', 'resource/ost.txt', 'resource/ost.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tentone',
    maintainer_email='tentone@outlook.com',
    description='DJI Tello control package for ROS 2',
    license='MIT',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'tello = tello.node:main',
            'simple_test = tello.simple_test:main',
            'ipn = tello.image_processing:main',
            'tcn = tello.taskc_node:main',
            'smn = tello.smc_node:main',
            'btn = tello.btc_node:main'
        ],
    },
)
