#!/bin/sh

python_changed=0
FILE_PATTERN=\.py$

git diff --cached --name-only | grep -q $FILE_PATTERN && python_changed=1

if [ $python_changed = 1 ]; then
  b5 python:lint
else
  echo "No python files in commit, skip python linting"
fi