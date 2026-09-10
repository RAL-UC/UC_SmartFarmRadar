from setuptools import find_packages
from setuptools import setup

setup(
    name='radar_msg',
    version='0.0.0',
    packages=find_packages(
        include=('radar_msg', 'radar_msg.*')),
)
