from setuptools import setup, find_packages

setup(
    name="robot_control",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.24.0",
        "pybullet>=3.2.5",
        "rich>=13.0.0",
    ],
)
