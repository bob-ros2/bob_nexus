from setuptools import find_packages
from setuptools import setup

setup(
    name='bob_topic_tools',
    version='1.0.0',
    packages=find_packages(
        include=('bob_topic_tools', 'bob_topic_tools.*')),
)
