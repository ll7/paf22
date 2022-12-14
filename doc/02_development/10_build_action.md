# GitHub actions

**Summary:** This page explains the GitHub build action we use to create an executable image of our work.

---

## Author

Tim Dreier

## Date

2.12.2022

## Table of contents

<!-- TOC -->
* [GitHub actions](#github-actions)
  * [Author](#author)
  * [Date](#date)
  * [General](#general)
  * [The Dockerfile (`build/docker/build/Dockerfile`)](#the-dockerfile--builddockerbuilddockerfile-)
  * [The build process](#the-build-process)
    * [1. Checkout repository (`actions/checkout@v3`)](#1-checkout-repository--actionscheckoutv3-)
    * [2. Set up Docker Buildx (`docker/setup-buildx-action@v2`)](#2-set-up-docker-buildx--dockersetup-buildx-actionv2-)
    * [3. Log in to the Container registry (`docker/login-action@v2`)](#3-log-in-to-the-container-registry--dockerlogin-actionv2-)
    * [4. Bump version and push tag (`mathieudutour/github-tag-action`)](#4-bump-version-and-push-tag--mathieudutourgithub-tag-action-)
      * [Example](#example)
    * [5. Build and push Docker image](#5-build-and-push-docker-image)
<!-- TOC -->

## General

The workflow defined in [`.github/workflows/build.yml`](../../.github/workflows/build.yml) creates an executable image which can later be submitted to the [CARLA leaderboard](https://leaderboard.carla.org) and pushes it to [GitHub Packages](ghcr.io).

The image can then be pulled with `docker pull ghcr.io/ll7/paf22:latest` to get the latest version or `docker pull ghcr.io/ll7/paf22:<version>` to get a specific version.

## The Dockerfile ([`build/docker/build/Dockerfile`](../../build/docker/build/Dockerfile))

The Dockerfile uses [Dockerfile+](https://github.com/edrevo/dockerfile-plus) to include the [agent Dockerfile](../../build/docker/agent/Dockerfile) to avoid duplicate code.
The code folder is then copied into the container instead of mounting it as in our dev setup.

## The build process

### 1. Checkout repository ([`actions/checkout@v3`](https://github.com/actions/checkout))

Trivial, just checks out the repo.

### 2. Set up Docker Buildx ([`docker/setup-buildx-action@v2`](https://github.com/docker/setup-buildx-action))

Set's up Buildx. This is needed to set up the correct driver to allow caching in step 5.

Detailed description why this is needed can be found [here](https://github.com/docker/build-push-action/issues/163#issuecomment-1053657228).

### 3. Log in to the Container registry ([`docker/login-action@v2`](https://github.com/docker/login-action))

Logs in with `GITHUB_TOKEN` into the registry (ghcr.io).

Example taken from [here](https://docs.github.com/en/actions/publishing-packages/publishing-docker-images)

### 4. Bump version and push tag ([`mathieudutour/github-tag-action`](https://github.com/mathieudutour/github-tag-action))

Creates a new tag with a [semantic version](https://semver.org/) number for the release.
The version number is determinated by the name of the commits in the release.

This is possible since [conventional commits](https://www.conventionalcommits.org/) are enforced by comlipy as described [here](./02_linting.md).

#### Example

| Commit message                                         | Release type  | Previous version number | New version number |
|--------------------------------------------------------|---------------|-------------------------|--------------------|
| fix(#39): build failing due to incorrect configuration | Patch Release | 0.0.1                   | 0.0.2              |
| feat(#39): Add automatic build process                 | Minor Release | 0.0.1                   | 0.1.0              |

Major releases can be done manually (e.g. `git tag v1.0.0`).

### 5. Build and push Docker image

Build and push the image to the registry. To avoid large downloads of the base image the [GitHub Actions cache](https://docs.docker.com/build/building/cache/backends/gha/)
is used to cache the image after build.
