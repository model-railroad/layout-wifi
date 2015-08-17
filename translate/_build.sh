#!/bin/bash
if [[ -z "$GOPATH" ]]; then . ./_setup.sh; fi
go build -v $@ src/translate.go && mv -v translate* bin/

