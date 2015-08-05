#!/bin/bash
set -e
set -x
cd $(dirname "$0")

# To remove a module: 
# 1- git submodule deinit $ROOT/$DIR ==> check .gitmodules, should have been removed from it
# 2- rm -rfv .git/modules/<path>     ==> just clearing internal caches
# 3- git rm --cached <path> (no trailing slash) ==> if not gives "already in index" when adding it back
#
# Also this conflicts with a git mv of the parent directory if not at the git root.

function checkout() {
    DIR=$1
    URL=$2
    ROOT=.
    GIT_USER=$(sed -n '/email = /s/.*= \(.*\)@.*/\1/p' ~/.gitconfig)
    if [[ -z $GIT_USER ]]; then set +x; echo "Git user not found"; exit 1; fi

    if [[ ! -d $ROOT/$DIR ]]; then
      git submodule add $URL $ROOT/$DIR
    fi

    git submodule update --init $ROOT/$DIR
}

checkout DigiFi https://github.com/digistump/DigiFi.git

checkout translate/src/github.com/stretchr/testify https://github.com/stretchr/testify.git
