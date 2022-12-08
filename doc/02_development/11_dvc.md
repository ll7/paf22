# Data Version Controls

**Summary:** This page describes what dvc is and how/where to use it.

---

## Author

Tim Dreier

## Date

8.12.2022

## Table of contents
<!-- TOC -->
* [Data Version Controls](#data-version-controls)
  * [Author](#author)
  * [Date](#date)
  * [Table of contents](#table-of-contents)
  * [General](#general)
  * [Installation](#installation)
  * [Common commands](#common-commands)
    * [Pull / push](#pull--push)
      * [Authentication](#authentication)
  * [Add file or folder to dvc](#add-file-or-folder-to-dvc)
  * [Python API](#python-api)
  * [Example](#example)
<!-- TOC -->

## General

DVC enables us to manage our datasets and trained models outside from git.
Instead of storing the full datasets or models inside git, dvc will just store a reference to the dataset or trained model stored in a separate storage.
This makes it possible to switch between versions of datasets and trained models and train models without touching the git repo.

![DVC flow](https://dvc.org/img/flow.gif)

(Taken from <https://dvc.org/>)

A great summary about what it does cant be found [here (german)](https://www.marianbiermann.de/data-version-control-reproduzierbares-machine-learning/).
Additional information can also be found in the according [GitHub repsoitory](https://github.com/iterative/dvc)

## Installation

Install dvc and the according gdrive extension by running `pip install dvc dvc-gdrive`.

## Common commands

### Pull / push

The commands to pull and push are similar to git. However, for pushing an authentication may be required.

```shell
# pull
dvc pull

# push
dvc push

# checkout (run after doing git checkout)
dvc checkout
```

#### Authentication

When pushing the first time you will receive a message similar to this:

```shell
Your browser has been opened to visit:

https://accounts.google.com/o/oauth2/auth?client_id=xxxxxxxxxxxx
```

To authenticate please click the link and login with your Google account.

> ❗️ Your Google account has to be added to the users with write access. Please contact [@timdreier](https://www.github.com/timdreier).

After this, you should get a success message similar to this in your browser:

```text
The authentication flow has completed.
```

Back in your shell, you should see that the push was successful.

## Add file or folder to dvc

To add a file or folder to dvc use the following commands:

```shell
# Add file data/data.xml
dvc add data/data.xml

# Add the whole data directory
dvc add data
```

## Python API

DVC also provides a great Python API, which makes it possible to use models stored in DVC directly.
This way, the model is automatically loaded during code execution, so developers who do not work directly with the models or datasets do not have to care about DVC.

```python
import dvc.api

with dvc.api.open(
    'get-started/data.xml',
    repo='https://github.com/iterative/dataset-registry'
) as f:
    # f is a file-like object which can be processed normally
```

([Source](https://dvc.org/doc/start/data-management/data-and-model-access#python-api))

More information can be found [here](https://dvc.org/doc/api-reference).

## Example

This example should give a short overview how to work with dvc. In this case it stores a dateset with dvc.
Storing a model file can be done the same way.

> The commands below are not meant to execute, since the example is already added in git.
> It should give a brief overview about how dvc works.
> However, the process is adaptable for any file or folder if you replace `doc/04_examples/dvc_example/dataset` with your path.

1. Add the folder `doc/04_examples/dvc_example/dataset` to dvc

   ```shell
   dvc add doc/04_examples/dvc_example/dataset
   ```

   > ❗️ if you already added the directory to git you have to remove it by running `git rm -r --cached 'doc/04_examples/dvc_example/dataset'`

2. Commit your changes in git

   ```shell
   git commit -m "feat(#83): Add dataset to dvc"
   ```

3. Push your changes to dvc

   ```shell
   dvc push
   ```
