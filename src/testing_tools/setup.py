from setuptools import setup, find_packages
setup(
    name = "testing_tools",
    version = "0.1",
    package_dir = {'': 'src',},
    package_data = {'': ['*.sh']},
    packages = find_packages('src'),
)
