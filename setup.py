from setuptools import setup, find_packages

setup(
    name="robot_control",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.24.0",
        "pybullet>=3.2.5",
        "rich>=13.0.0",
        "pin>=3.9.0",
        "fastapi>=0.110",
        "uvicorn>=0.29",
    ],
)
