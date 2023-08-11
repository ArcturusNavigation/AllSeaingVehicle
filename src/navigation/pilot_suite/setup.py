from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
a = generate_distutils_setup(
    packages=['pilot_suite'],
    package_dir={'': 'src'}
    )
setup(**a)
