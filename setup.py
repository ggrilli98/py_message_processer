from setuptools import setup

package_name = 'py_message_processer'

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
    maintainer='gregorio',
    maintainer_email='gregorio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [   [
                'spreader controller = py_message_processer.Speaker_node:main',
                'processer = py_message_processer.Talkback_node:main',
                'geometrical_transformer = py_message_processer.joint_modeler:main',
        ],
        ],
    },
)
