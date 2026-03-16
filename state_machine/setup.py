from setuptools import find_packages, setup

package_name = 'state_machine'
info_version = '0.0.0'
info_autor = 'Diego Muñoz Rojas'
info_email = 'dammr@uc.cl'
info_descripcion = 'Máquina de estado del sistema bunker, radar, PTU y GPS'
info_licencia = 'MIT License'

setup(
    name=package_name,
    version=info_version,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools'
    ],
    zip_safe=True,
    maintainer=info_autor,
    maintainer_email=info_email,
    description=info_descripcion,
    license=info_licencia,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = state_machine.state_machine:main',
            'active_radar = state_machine.active_radar:main',
            'active_ptu_radar = state_machine.active_ptu_radar:main',
        ],
    },
)
