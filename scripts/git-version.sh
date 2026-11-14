#!/bin/bash

HASH=$(git rev-parse --short=8 HEAD 2>/dev/null)

if [ -z "$HASH" ]; then
    HASH="00000000"
fi

cat > git-version.h <<EOT
#pragma once
#define GIT_VERSION 0x${HASH}
EOT
