# Git Style

**Summary:** This page gives an overview over different types of git workflows to choose from.


---
### Author

Josef Kircher

### Date

07.11.2022

### Prerequisite

Git Flow:

sudo apt-get install git-flow

---
[[TOC]] 

# Git workflows

## Git flow

<img src="../assets/04 Hotfix branches.svg" alt="Git Feature" width="500" style="margin-left:30%"/>

### Branch strategy

---

Different types of branches for different types of tasks.

Branches include:

- Main: holds the current latest version of the project
- Develop: this branch is used for development and the base for releases
- Feature: for every feature a branch is created from Develop
- Release: for every release version a separat release branch is created
- Hotfix: to hotfix already released code this branch is used

### Pros

---
1. The various types of branches make it easy and intuitive to organize your work.
2. The systematic development process allows for efficient testing.
3. The use of release branches allows you to easily and continuously support multiple versions of production code. 
4. support from commandline tool git-flow to work with this workflow

### Cons

---
1. Depending on the complexity of the product, the Git flow model could overcomplicate and slow the development process and release cycle.
2. Because of the long development cycle, Git flow is historically not able to support Continuous Delivery or Continuous Integration.
3. Feature branches are designated to live until the feature is completely done, might be longer than a sprint, solution for PRs might be needed
4. the use of PRs is not natively considered in this workflow model, but should be easily adaptable

### Resources

---

[Git Flow Cheat Sheet](https://danielkummer.github.io/git-flow-cheatsheet/)



## Git Feature Branch (GitHub Flow)
<img src="../assets/git-flow.svg" alt="Git Feature" width="500" style="margin-left:30%"/>

### Branch strategy

---

Two types of branches:

- Main: contains production ready code
- Feature: new branches based on main for each feature

### Pros

---
1. Very simple
2. allows for Continuous Delivery and Continuous integration
3. works great for small teams

### Cons

---
1. This Git branch strategy is unable to support multiple versions of code in production at the same time.
2. The lack of dedicated development branches makes GitHub flow more susceptible to bugs in production.

# Git style

## Branch naming

---
Feature branch: #(issue number)-short-description-of-issue      (separator: '-')

## Commit messages
---
- proceed to [Commit Messages](./03_commit.md)

## Git commands cheat sheet
---
https://education.github.com/git-cheat-sheet-education.pdf

# Sources
https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow

https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow

https://www.gitkraken.com/learn/git/best-practices/git-branch-strategy