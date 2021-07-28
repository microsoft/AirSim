import setuptools
from airsim import __version__

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="airsim",
    version=__version__,
    author="Shital Shah",
    author_email="shitals@microsoft.com",
    description="Open source simulator based on Unreal Engine for autonomous vehicles from Microsoft AI & Research",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/microsoft/airsim",
    packages=setuptools.find_packages(),
	license='MIT',
    classifiers=(
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ),
    install_requires=[
          'msgpack-rpc-python', 'numpy', 'opencv-contrib-python'
    ]
)
