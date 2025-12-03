from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'spot_tennis_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andon Breitenfeld',
    maintainer_email='andonbreitenfeld@utexas.edu',
    description='Demo for Spot to detect, navigate to, and manipulate tennis balls.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ball_selector = spot_tennis_demo.ball_selector:main',
        ],
    },
)
