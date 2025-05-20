from setuptools import setup
import os
from glob import glob

package_name = "jmbot"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TuNombre",
    maintainer_email="tunombre@email.com",
    description="Descripción de tu paquete",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [

       'dh= jmbot.scripts.dh:main',
       'raton = jmbot.scripts.raton:main', 
       'sub_terminal=jmbot.sub_terminal:main'],
    },
)

