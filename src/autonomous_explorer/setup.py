from setuptools import setup

package_name = 'autonomous_explorer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/explorer.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/robot_params.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'bresenham'],
    zip_safe=True,
    maintainer='z',
    maintainer_email='z@todo.todo',
    description='Autonomous exploration package for mobile robots using frontier-based exploration with Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = autonomous_explorer.myexplorer:main',
            'detect_node = autonomous_explorer.detect_and_grabV1:main', # 创建了可执行文件，运行myexplorer.py中的main函数
        ],
    },
)
