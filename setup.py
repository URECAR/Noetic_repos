from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ur10e_2f_140_control'],
    package_dir={'': 'scripts'}
)

setup(**d)