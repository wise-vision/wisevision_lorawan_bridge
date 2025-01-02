#!/bin/bash

CHIRPSTACK_VERSION="4.8.1"

if [ -e proto ]; then
    exit 0
fi
mkdir -p proto/chirpstack_api

wget -q https://github.com/chirpstack/chirpstack/archive/refs/tags/v${CHIRPSTACK_VERSION}.tar.gz
tar -xf v${CHIRPSTACK_VERSION}.tar.gz

cp -r chirpstack-${CHIRPSTACK_VERSION}/api/proto/* proto/chirpstack_api

sed -i 's@^import "common/@import \"chirpstack_api/common/@g' `find proto/chirpstack_api -type f -name "*.proto*"`
sed -i 's@^import "gw/@import \"chirpstack_api/gw/@g' `find proto/chirpstack_api -type f -name "*.proto*"`
sed -i 's@^import "api/@import \"chirpstack_api/api/@g' `find proto/chirpstack_api -type f -name "*.proto*"`
sed -i 's@^import "google/api/@import \"chirpstack_api/google/api/@g' `find proto/chirpstack_api -type f -name "*.proto*"`

rm -rf chirpstack-${CHIRPSTACK_VERSION} v${CHIRPSTACK_VERSION}.tar.gz
