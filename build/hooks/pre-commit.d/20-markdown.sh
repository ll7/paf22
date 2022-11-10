#!/bin/sh

markdown_changed=0
FILE_PATTERN=\.md$

git diff --cached --name-only | grep -q $FILE_PATTERN && markdown_changed=1

if [ $markdown_changed = 1 ]; then
  b5 lint
else
  echo "No markdown files in commit, skip markdown linting"
fi