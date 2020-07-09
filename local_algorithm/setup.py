from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['local_algorithm'],
    scripts=['scripts/main.py','scripts/vis_driver.py'],
    package_dir={'': 'src'}
)

setup(**d)