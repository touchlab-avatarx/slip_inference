# Copyright (c) 2022 Touchlab Limited. All Rights Reserved
# Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "slip_inference"
package_description = "An inference package for touch models"
maintainer_name = "vishal"
maintainer_email = "vishal@touchlab.io"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*.*")),
        (os.path.join("share", package_name, "model"), glob("models/*.*"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer=maintainer_name,
    maintainer_email=maintainer_email,
    description=package_description,
    license="Touchlab Limited",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'inference_node = slip_inference.inference_node:main', 
        ],
    },
)
