#!/bin/bash
if [[ -z "$GOPATH" ]]; then . ./_setup.sh; fi
go test -v src/translate/*.go
