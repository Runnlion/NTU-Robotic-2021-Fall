from setuptools import setup

package_name = 'send_script'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Robotics HW4: Object stacking',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_script=send_script.send_script:main',
            'img_sub=send_script.image_sub:main'
            'talker = send_script.publisher_member_function:main',
            'listener = send_script.subscriber_member_function:main',
            'vision_script = send_script.vision:main',
        ],
    },
)
