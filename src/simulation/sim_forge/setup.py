from glob import glob
from setuptools import setup

setup(
    name="sim_forge", version="0.1.0", packages=["sim_forge"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sim_forge"]),
        ("share/sim_forge", ["package.xml"]),
        ("share/sim_forge/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "simforge_probe = sim_forge.tools:probe",
        "simforge_smoke = sim_forge.tools:smoke",
        "simforge_ros_bridge = sim_forge.bridge:main",
        "simforge_diagnostic_driver = sim_forge.diagnostic:main",
        "simforge_score = sim_forge.score:main",
    ]},
)
