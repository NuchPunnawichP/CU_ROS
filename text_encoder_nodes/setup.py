from setuptools import find_packages, setup

package_name = 'text_encoder_nodes'

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
    maintainer='komert',
    maintainer_email='Nuch.Punnawich.P@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        'text_encode_server = text_encoder_nodes.text_encode_server:main',
            'turtle_pose_service = text_encoder_nodes.turtle_pose_service:main',
        ],
    },
)
