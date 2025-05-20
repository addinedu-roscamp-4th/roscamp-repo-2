import os
from setuptools import setup, find_packages, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'cookM'

# 检查 setuptools 版本
use_dash_separated_options = Version(setuptools_version) < Version("58.0.0")


# 动态生成 setup.cfg 内容
setup_cfg_content = """
[develop]
{script_option}=$base/lib/{package_name}

[install]
{install_scripts_option}=$base/lib/{package_name}
""".format(
    package_name=package_name,
    script_option='script-dir' if use_dash_separated_options else 'script_dir',
    install_scripts_option='install-scripts' if use_dash_separated_options else 'install_scripts'
)

# 将内容写入 setup.cfg
with open("setup.cfg", "w") as f:
    f.write(setup_cfg_content)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),  # 🔹 cookM 하위 모듈 자동 포함
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🔹 리소스 파일 등록
        (os.path.join('share', package_name), [
            'cookM/trajectories.yaml',
            'cookM/calibration_refined4.npz',
            'cookM/calibration_aa48.npz',
            'cookM/best.pt'
        ]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='Multi-robot cooking motion planning system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_broadcaster = cookM.nodes.pose_broadcaster:main',
            'tcp_interface = cookM.tcp_interface:main',
            'cookgpt = cookM.cookgpt:main',
            'cook_comp_test = cookM.cook_comp_test:main',
            'unit_test = cookM.unit_test:main',
            'gripper_distance_node = cookM.gripper_distance_node:main',
        ],
    },
)