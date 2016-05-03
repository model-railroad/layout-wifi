#!/bin/bash
if [[ -z "$GOPATH" ]]; then . ./_setup.sh; fi
if [[ ! -d "bin" ]]; then mkdir -p bin; fi
go build -v $@ src/translate.go && for T in translate translate.exe ; do \
    if [[ -f $T ]]; then mv -v $T bin/ ; fi ; done

