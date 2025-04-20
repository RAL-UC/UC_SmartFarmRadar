from setuptools import find_packages, setup

package_name = 'radar_msg'
info_version = '0.0.0'
info_autor = 'Diego Muñoz Rojas'
info_email = 'dammr@uc.cl'
info_descripcion = 'Paquete para generar mensajes personalizados de radar'
info_licencia = 'TODO: License declaration'

setup(
    name=package_name,
    version=info_version,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial',],
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
            'publish_radar_data = scripts.publish_radar_data:main',
            'subscribe_radar_data = scripts.subscribe_radar_data:main',
        ],
    },
)

