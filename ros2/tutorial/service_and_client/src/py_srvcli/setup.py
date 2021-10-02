from setuptools import setup

package_name = 'py_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yubo',
    maintainer_email='yubo.upt@gmail.com',
    description='Python client server tutorial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
        'client_github = py_srvcli.client_member_function_github:main',
        'service_custom_interface = py_srvcli.service_member_function_custom_interface:main',
        'client_custom_interface = py_srvcli.client_member_function_custom_interface:main',
        ],
    },
)
