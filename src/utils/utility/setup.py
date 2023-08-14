from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
a = generate_distutils_setup(
    packages=['utility'],
    package_dir={'': 'src'}
    )
setup(**a)
