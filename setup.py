import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
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
    install_requires=['rospy', 'numpy', 'tf2_ros', 'tf.transformations'] # TODO: double check this is how requirements work, and that these are the correct package names.
)