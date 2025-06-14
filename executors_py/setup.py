from setuptools import find_packages, setup

package_name = "executors_py"

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
    description="My python executors package",
    license="Unlicensed",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "single_threaded_executor = executors_py.single_threaded_executor:main",
            "multi_threaded_executor = executors_py.multi_threaded_executor:main",
        ],
    },
)
