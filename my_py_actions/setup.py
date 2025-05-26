from setuptools import find_packages, setup

package_name = "my_py_actions"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="beal",
    maintainer_email="guibeal@guibeal.com",
    description="My python actions",
    license="Unlicensed",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "count_until_server = my_py_actions.count_until_server:main",
            "count_until_client = my_py_actions.count_until_client:main",
        ],
    },
)
