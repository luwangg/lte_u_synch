from setuptools import setup, find_packages


def readme():
    with open('README.md') as f:
        return f.read()


setup(
    name='lte_u_synch',
    version='0.1.0',
    packages=find_packages(),
    scripts=[],
    url='',
    license='MIT',
    author='Piotr Gawlowicz',
    author_email='gawlowicz@tkn.tu-berlin.de',
    description='Cross-Technology Detector',
    long_description='Cross-Technology Detector',
    keywords='cross-technology synchronization',
    install_requires=['pyzmq', 'numpy', 'PyCRC', 'IPython', 'sh', 'PyRIC', 'peakutils'],
    extras_require={},
)
