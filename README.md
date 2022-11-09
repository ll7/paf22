# Praktikum Autonomes Fahren 2022 - PAF22

## Prerequisites

To be able to execute and develop the project, you need a Linux system equipped with: - NVIDIA GPU (everything >= RTX 2060 should be fine) - A minimum of 16G of RAM - A minimum of 100G of free disk space

As the project is still in early development, these requirements are subject to change.

Additionally, you need to install the following dependencies:

## Installation

To run the project you have to install [b5](https://github.com/team23/b5) and [docker](https://docs.docker.com/engine/install/) with NVIDIA GPU support, [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker). More detailed instruction about setup and execution can be found [here](./doc/general/01_overview.md)

```shell
# Install b5
pip install b5

# Setup project
b5 install

# Run project
b5 run
```

More available b5 commands are documented [here](./doc/general/03_commands.md).

## Development

If you contribute to this project please read the guidelines first. They can be found [here](./doc/developement/01_overview.md).
