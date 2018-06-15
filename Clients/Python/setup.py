import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="airsim",
    version="1.2.0",
    author="Shital Shah",
    author_email="sytelus@gmail.com",
    description="Open source simulator based on Unreal Engine for autonomous vehicles from Microsoft AI & Research",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/microsoft/airsim",
    packages=setuptools.find_packages(),
    classifiers=(
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ),
)