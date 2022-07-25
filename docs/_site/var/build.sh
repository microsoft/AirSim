#!/usr/bin/env bash
apt-get update && \
apt-get install -y libfontconfig zip npm git apt-transport-https ca-certificates curl openssl && \
npm i -g npm && \
npm cache clean -f && \
npm install -g n && \
n stable && \
node --version && \
npm --version && \
bundle install