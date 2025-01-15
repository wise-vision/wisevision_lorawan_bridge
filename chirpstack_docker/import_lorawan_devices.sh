#!/usr/bin/env bash
#
#  Copyright (C) 2025 wisevision
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

docker compose run --rm --entrypoint sh --user root chirpstack -c '\
    apk add --no-cache git && \
    git clone https://github.com/brocaar/lorawan-devices /tmp/lorawan-devices && \
    chirpstack -c /etc/chirpstack import-legacy-lorawan-devices-repository -d /tmp/lorawan-devices'

