import setuptools

setuptools.setup(
    name="carla_cyber_bridge",
    version="0.0.1",
    author="Angel Avila",
    author_email="angel.avila@ridecell.com",
    description="Cyber version of Carla ros-bridge.",
    url="https://github.com/auroai/carla_apollo_bridge",
    packages=setuptools.find_packages(),
    python_requires='>=2.7',
    install_requires=[
        'numpy',
        'opencv-python',
        'protobuf',
        'pygame',
        'pyproj',
        'pyyaml',
    ]
)
