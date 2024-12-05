import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'week5_model'

import os
import os
from glob import glob

def mesh_data_files(source_dir, target_dir, pattern="**/*.stl"):
    """
    Collect files matching the pattern and preserve their relative folder structure.

    Args:
        source_dir (str): Root directory to search for files.
        target_dir (str): Destination base directory for files.
        pattern (str): Glob pattern for file search (default: "**/*.stl").

    Returns:
        list: List of tuples suitable for `data_files` in setup.py.
    """
    data_files = []
    
    # Search for files matching the pattern
    for filepath in glob(os.path.join(source_dir, pattern), recursive=True):
        if os.path.isfile(filepath):  # Ensure it's a file
            # Compute relative path from the source directory
            relative_path = os.path.relpath(filepath, source_dir)
            # Create the target installation path
            install_path = os.path.join(target_dir, os.path.dirname(relative_path))
            # Append to the data files list
            data_files.append((install_path, [filepath]))
    
    return data_files


mesh_files = mesh_data_files('mesh', os.path.join('share', package_name, 'mesh'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        *mesh_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaenote',
    maintainer_email='na06219@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
