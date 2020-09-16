#!/usr/bin/env bash
set -e

export GIT_COMMITTER_NAME="Vehicle Researcher"
export GIT_COMMITTER_EMAIL="user@comma.ai"
export GIT_AUTHOR_NAME="Vehicle Researcher"
export GIT_AUTHOR_EMAIL="user@comma.ai"

SOURCE_DIR=$PWD
TARGET_DIR=tmp_op_ci

BRANCH=r2-ci-tests

mkdir $TARGET_DIR && cd $TARGET_DIR

git init
git remote add origin https://github.com/jamcar23/openpilot.git
git checkout --orphan "$BRANCH"

find . -maxdepth 1 -not -path './.git' -not -name '.' -not -name '..' -exec rm -rf '{}' \;

cd $SOURCE_DIR

cp -pR --parents $(cat release/files_common) $TARGET_DIR/

cd $TARGET_DIR

git add -A
git commit -am "release: $(date +%s)"
#git push -uf origin "$BRANCH"
