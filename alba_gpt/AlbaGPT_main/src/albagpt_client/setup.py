from setuptools import find_packages, setup

package_name = 'albagpt_client'

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
    maintainer='addinedu',
    maintainer_email='jungseonglian@sju.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_reciever = albagpt_client.topic_reciever:main',
            'llm_reciever = albagpt_client.llm_response_reciever:main'
        ],
    },
)
