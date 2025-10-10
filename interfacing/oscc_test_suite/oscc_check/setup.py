
import sys
from setuptools import setup, find_packages
from setuptools.command.test import test as TestCommand

long_description = open('README.md', 'r').read()


class PyTest(TestCommand):
    user_options = [("pytest-args=", "a", "Arguments to pass to pytest")]

    def initialize_options(self):
        TestCommand.initialize_options(self)
        self.pytest_args = ""

    def run_tests(self):
        import shlex

        # import here, cause outside the eggs aren't loaded
        import pytest

        errno = pytest.main(shlex.split(self.pytest_args))
        sys.exit(errno)


setup(name='oscc-check',
      version='0.0.1',
      url='https://github.com/PolySync/oscc-check',
      author='Shea Newton',
      author_email='snewton@polysync.io',
      maintainer='PolySync Technologies',
      maintainer_email='help@polysync.io',
      description='Check that your vehcile and the installed OSCC are in a good state.',
      long_description=long_description,
      download_url='https://github.com/PolySync/oscc-check',
      packages=["oscccan"],
      license='MIT',
      install_requires=[
          'colorama',
          'docopt',
          'python-can',
      ],
      scripts=['oscc-check.py'],
      tests_require=['pytest', 'hypothesis'],
      test_suite="tests",
      cmdclass={"test": PyTest},
      classifiers=[
          'Environment :: Console',
          'License :: MIT License',
          'Natural Language :: English',
          'Operating System :: Linux',
          'Programming Language :: Python',
          'Topic :: Scientific/Engineering',
      ],
      )
