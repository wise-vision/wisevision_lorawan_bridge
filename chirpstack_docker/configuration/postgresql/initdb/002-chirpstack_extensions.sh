#!/bin/bash
#
#  Copyright (C) 2025 wisevision
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
set -e

psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" --dbname="chirpstack" <<-EOSQL
    create extension pg_trgm;
    create extension hstore;
EOSQL
