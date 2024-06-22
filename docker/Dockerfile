# Basic Dockerfile for testing compilation in Linux
FROM ubuntu:20.04

ENV TZ=America/Indiana/Indianapolis

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && \
    apt-get install -y \
    git \
    vim \
    sudo

WORKDIR /root/workspace

CMD [ "/bin/bash" ]