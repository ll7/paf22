# Data Version Controls

**Summary:** This page describes what dvc is and how/where to use it.

---

## Author

Tim Dreier

## Date

7.12.2022

## Installation

Install dvc by running `pip install dvc`.

## Common commands

### Pull / push

The commands to pull and push are similar to git:

```shell
# pull
dvc pull

# push
dvc push
```

## Add file or folder to dvc

To add a file or folder to dvc use the following commands:

```shell
# Add file data/data.xml
dvc add data/data.xml

# Add the whole data directory
dvc add data
```
