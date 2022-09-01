import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='ROS_utils',
    version='0.0.1',
    author='Quinn Bowers',
    author_email='qbowers@mit.edu',
    description='Handy ROS utility functions and classes',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://github.com/Darbeloff/ROS_utils',
    project_urls = {
        "Bug Tracker": "https://github.com/Darbeloff/ROS_utils/issues"
    },
    license='MIT',
    packages=['ROS_utils'],
    install_requires=['rospy', 'numpy'],

    test_suit='nose.collector',
    tests_require=['nose', 'rospy', 'numpy']
)