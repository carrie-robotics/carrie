from setuptools import find_packages, setup

package_name = 'carrie_perception'

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
    maintainer='laibazahid',
    maintainer_email='laibazahid26@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sam3_detection = carrie_perception.sam3_node:main',
            'test_publisher = carrie_perception.test_publisher:main',
        ],
    },
)
