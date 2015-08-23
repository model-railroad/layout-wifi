#!/bin/bash
if [[ -z "$GOPATH" ]]; then . ./_setup.sh; fi
if [[ ! -d "bin" ]]; then mkdir -p bin; fi
go build -v $@ src/translate.go && mv -v translate* bin/

