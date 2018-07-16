#!/bin/bash

readonly UPSTREAM_REPO="https://git.code.sf.net/p/libmtp/code"

echo "Add upstream remote: ${UPSTREAM_REPO}"
git remote add upstream ${UPSTREAM_REPO}
git fetch upstream --no-tags

# LibMTP only has a "master" branch
echo "Setup upstream branches"
git branch upstream-master upstream/master --track
