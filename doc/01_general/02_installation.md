# 🛠️ Installation

To run the project you have to install [b5](https://github.com/team23/b5) and [docker](https://docs.docker.com/engine/install/) with NVIDIA GPU support, [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

For development, we further recommend PyCharm Professional. More information about its installation can be found in [PyCharm Setup](../02_development/06_pycharm_setup.md)

## Installation

If not yet installed first install b5 and docker as described in section [b5 installation](#b5-installation) and [Docker with NVIDIA GPU support](#docker-with-nvidia-gpu-support).

After that setting up the project and executing it is as easy as that:

```shell
# Setup project
b5 install

# Run project
b5 run
```

## b5 installation

Make sure you have installed python and pip. If not yet installed, you can do by the following (Ubuntu):

```shell
# Install python3
sudo apt install python3
```

Afterwards just install b5 by running:

```shell
# Install b5
pip install b5
```

## Docker with NVIDIA GPU support

For this installation, it's easiest to follow the guide in the [NVIDIA docs](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

For simplicity, we list the necessary steps here:

### Docker

Install Docker using the convenience script.

```shell
curl https://get.docker.com | sh \
 && sudo systemctl --now enable docker
```

### Allow non-root user to execute Docker commands

We recommend this step, to not have to execute every command as root.

```shell
# add docker group
sudo groupadd docker

# add your current user to the docker group
sudo usermod -aG docker $USER
```

After this, _restart_ your system to propagate the group changes.

### NVIDIA Container toolkit

Setup the package repository and the GPG key:

```shell
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Install the `nvidia-docker2` package (and dependencies) after updating the package listing:

```shell
sudo apt-get update
sudo apt-get install -y nvidia-docker2
```

Restart the Docker daemon to complete the installation after setting the default runtime:

```shell
sudo systemctl restart docker
```

## 🚨 Common Problems

### `b5: command not found`

---

1. If you already installed b5 (`pip install b5`) try to log out and log back in.

2. If that doesn't help add this line to your `~/.bash_profile` or `~/.bashrc`:

    ```shell
    export PATH=$PATH:$HOME/.local/bin
    ```

([possible reason](https://stackoverflow.com/a/73256004))
