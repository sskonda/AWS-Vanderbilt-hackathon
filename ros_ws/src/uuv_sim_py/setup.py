from setuptools import setup

package_name = 'uuv_sim_py'

setup(
    name=package_name,                 # MUST be uuv_sim_py (underscore)
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikhil',
    maintainer_email='nikhil',
    description='Mini sim for UUV swarm',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim = uuv_sim_py.sim:main',   # package.module:function
        ],
    },
)
