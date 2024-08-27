from setuptools import find_packages, setup

package_name = 'qr_code_finder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthonyubuntu',
    maintainer_email='anthony.moundalak@net.usj.edu.lb',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'qr_code_detector = qr_code_finder.qr_code_detector:main',
        ],
    },
)
