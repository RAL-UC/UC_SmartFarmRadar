from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'radar_package'
info_version = '0.0.0'
info_autor = 'Diego Muñoz Rojas'
info_email = 'dammr@uc.cl'
info_descripcion = 'Paquete que contiene los nodos de procesamiento y adquisición de datos del radar'
info_licencia = 'TODO: License declaration'

setup(
    name=package_name,
    version=info_version,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.pkl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'radar_msg',
        'numpy',
        'adi',
    ],
    zip_safe=True,
    maintainer=info_autor,
    maintainer_email=info_email,
    description=info_descripcion,
    license=info_licencia,
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'radar_sync_node = radar_package.radar_sync_node:main',
            'radar_node = radar_package.radar_node:main',
            'process_data_node = radar_package.process_data_node:main',
            'visulizacion_node = radar_package.visualizacion:main',
            'data_processing_node = radar_package.data_processing_node:main',
            'mosaic_node = radar_package.mosaic:main',
            'mapa_cartesiano = radar_package.mapa_cartesiano:main'
        ],
    },
)
