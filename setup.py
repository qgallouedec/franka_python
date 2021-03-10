from setuptools import setup, find_packages

with open('README.md', 'r') as f:
    long_description = f.read()

setup(
    name='franka_gym',
    description='OpenAI Gym environment for real Franka Emika Panda robot',
    author='Quentin GALLOUÉDEC',
    author_email='gallouedec.quentin@gmail.com',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/qgallouedec/franka_gym',
    packages=find_packages(),
    version='0.0.0',
    install_requires=['rospkg', 'defusedxml', 'netifaces', 'opencv-python']
)