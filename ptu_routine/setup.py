from setuptools import find_packages, setup

package_name = 'ptu_routine'
info_version = '0.0.0'
info_autor = 'Diego Muñoz Rojas'
info_email = 'dammr@uc.cl'
info_descripcion = 'Paquete que contiene nodos para controlar el PTU-C46 haciendo un barrido en ángulos predefinidos'
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
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer=info_autor,
    maintainer_email=info_email,
    description=info_descripcion,
    license=info_licencia,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptu_routine_node = ptu_routine.ptu_routine_node:main',
        ],
    },
)
