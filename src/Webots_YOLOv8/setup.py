import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'Webots_YOLOv8'
__version__ = '0.0.1'

setup(
    name=package_name,
    version=__version__,
    packages=find_packages(exclude=['docs', 'test', 'test.*', 'tests', 'tests.*']),

    # data_files informa ao colcon quais arquivos adicionais instalar.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'modelo'),
            glob('modelo/*.pt') + glob('modelo/*.onnx')),
        (os.path.join('share', package_name, 'recursos'), glob('recursos/*.csv')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],

    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pandas',
        'scipy',
        'ultralytics',
    ],
    python_requires='>=3.8',
    zip_safe=False,
    description='Pacote de detecção de objetos para a EDROM',
    long_description='Este programa detecta bola, robôs e outros elementos do campo de futebol de robôs.',
    license='MIT',

    entry_points={
        'console_scripts': [
            'finder = Webots_YOLOv8.yolo_simulation:main',
        ],
    },
    author='IVANj',
)
