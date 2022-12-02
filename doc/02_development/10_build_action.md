# Github actions

**Summary:** This page explains the GitHub build action we use to create an executable image of our work.

---

## Author

Tim Dreier

## Date

2.12.2022

## General

### Steps

#### 1. Checkout repository ([`actions/checkout@v3`](https://github.com/actions/checkout))

Trivial, just checks out the repo.

#### 2. Set up Docker Buildx ([`docker/setup-buildx-action@v2`])

Set's up Buildx. Needed that caching in step 5 works.

#### 3. Log in to the Container registry

#### 4. Extract metadata (tags, labels) for Docker

#### 5. Build and push Docker image
