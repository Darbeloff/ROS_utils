import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='ros_utils',
    version='0.0.1',
    author='Quinn Bowers',
    author_email='qbowers@mit.edu',
    description='Handy ROS utility functions and classes',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://github.com/Darbeloff/ros_utils',
    project_urls = {
        "Bug Tracker": "https://github.com/Darbeloff/ros_utils/issues"
    },
    license='MIT',
    packages=['ros_utils'],
    install_requires=['numpy'],

    test_suit='nose.collector',
    tests_require=['nose', 'numpy']
)