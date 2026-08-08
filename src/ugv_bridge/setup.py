import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ugv_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        # El .xacro es la fuente; el .urdf plano es el generado (scripts/generar_urdf.sh).
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        
        # --- AQUÍ ESTÁ LA LÍNEA MÁGICA PARA EL MODELO 3D DE DARPA ---
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        # -----------------------------------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benja',
    maintainer_email='benja@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'flipper_node = ugv_bridge.flipper_node:main',
            'track_odometry_node = ugv_bridge.track_odometry_node:main',
            'motor_emulator = ugv_bridge.motor_emulator:main',
            # 'gui_a_vcan' y 'dummy_rl_controller' apuntaban a módulos inexistentes
            # (fallaban al ejecutarse). Los reemplazan los dos de abajo, que sí existen.
            'gui_a_comandos = ugv_bridge.gui_a_comandos:main',
            'can_monitor = ugv_bridge.can_monitor:main',
            'kinematic_guardian = ugv_bridge.kinematic_guardian:main',
            'pi3hat_imu = ugv_bridge.pi3hat_imu_node:main',
        ],
    },
)
