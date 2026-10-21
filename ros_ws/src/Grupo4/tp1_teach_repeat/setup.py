from glob import glob
from setuptools import setup

setup(
    name='tp1_teach_repeat', version='0.1.0',
    packages=['tp1_teach_repeat'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tp1_teach_repeat']),
        ('share/tp1_teach_repeat', ['package.xml']),
        ('share/tp1_teach_repeat/launch', glob('launch/*.launch.py')),
        ('share/tp1_teach_repeat/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='Grupo 4', maintainer_email='grupo4@example.com',
    description='Teach and Repeat diferencial con encoders e ICP planar.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'localization = tp1_teach_repeat.localization:main',
        'controller = tp1_teach_repeat.controller:main',
        'teleop = tp1_teach_repeat.teleop:main',
    ]},
)
