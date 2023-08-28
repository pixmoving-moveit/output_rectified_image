from setuptools import setup

package_name = 'output_rectified_image_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', [f'launch/image_rectified.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zymouse',
    maintainer_email='zymouse@pixmoving.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['output_rectified_image_py_node = output_rectified_image_py.undistorted_image_node:main'
        ],
    },
)
