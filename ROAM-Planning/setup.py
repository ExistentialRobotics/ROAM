## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['utilities',
              'footprints',
              'planners',
              'mapping',
              'sensors'],
    package_dir={'utilities': 'src/utilities',
                 'footprints': 'src/footprints',
                 'planners': 'src/planners',
                 'mapping': 'src/mapping',
                 'sensors': 'src/sensors'}
)
setup(**setup_args)

