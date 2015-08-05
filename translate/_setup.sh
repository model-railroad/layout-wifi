#!/bin/bash
_BS=$(dirname "$BASH_SOURCE")
if [[ -d "$_BS" ]]; then
    export GOPATH=$(readlink -f $PWD/$_BS)
    if [[ -x $(which cygpath) ]]; then
        export GOPATH=$(cygpath -w "$GOPATH")
    fi
    echo "GOROOT=$GOROOT"
    echo "GOPATH=$GOPATH"
fi
