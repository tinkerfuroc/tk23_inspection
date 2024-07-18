from setuptools import find_packages, setup

package_name = 'robocup_tasks'

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
    maintainer='a',
    maintainer_email='cdsfcesf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carry_my_luggage = robocup_tasks.carry_my_luggage:main',
            'receptionist = robocup_tasks.receptionist:main',
            'tidy_up = robocup_tasks.tidy_up:main',
            'serve_breakfast = robocup_tasks.serve_breakfast:main',
            'inspection = robocup_tasks.inspection:main'
        ],
    },
)
