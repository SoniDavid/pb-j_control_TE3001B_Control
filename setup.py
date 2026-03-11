from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'puzzlebot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PuzzleBot Team',
    maintainer_email='team@puzzlebot.dev',
    description='Nonholonomic control benchmarking for PuzzleBot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── Simulation ──
            'puzzlebot_sim       = puzzlebot_control.puzzlebot_sim:main',
            'terrain_perturb     = puzzlebot_control.terrain_perturbation:main',
            # ── Controllers ──
            'pid_controller      = puzzlebot_control.pid_controller:main',
            'smc_controller      = puzzlebot_control.smc_controller:main',
            'ismc_controller     = puzzlebot_control.ismc_controller:main',
            'ctc_controller      = puzzlebot_control.ctc_controller:main',
            'ph_controller       = puzzlebot_control.ph_controller:main',
            # ── Teleop ──
            'teleop_keyboard     = puzzlebot_control.teleop_keyboard:main',
            # ── Dashboard ──
            'dashboard           = puzzlebot_control.dashboard:main',
            # ── Benchmark ──
            'lyapunov_benchmark  = puzzlebot_control.lyapunov_benchmark:main',
        ],
    },
)
