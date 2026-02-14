from setuptools import find_packages
from setuptools import setup

setup(
    name='bob_llm',
    version='1.0.2',
    packages=find_packages(
        include=('bob_llm', 'bob_llm.*')),
)
