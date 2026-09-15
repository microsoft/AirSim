FROM nvidia/cudagl:9.0-devel

RUN apt-get update && apt-get install -y --no-install-recommends \
	sudo \
	libglu1-mesa-dev \
	xdg-user-dirs \
	pulseaudio \
	x11-xserver-utils \
	unzip \
	wget \
	software-properties-common \
	ca-certificates \
	&& rm -rf /var/lib/apt/lists/*

# python3.6 is no longer available (issue #9778). Use deadsnakes Python 3.8 when present;
# otherwise fall back to the distro python3 package.
RUN apt-get update \
	&& (apt-add-repository -y ppa:deadsnakes/ppa || true) \
	&& apt-get update \
	&& (apt-get install-y python3.8 python3.8-distutils \
		|| apt-get install -y python3 python3-pip) \
	&& if command -v python3.8 >/dev/null 2>&1; then \
		update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1 || true; \
		python3.8 -m pip install --upgrade pip setuptools wheel || python3 -m pip install --upgrade pip setuptools wheel; \
	else \
		python3 -m pip install --upgrade pip setuptools wheel; \
	fi \
	&& rm -rf /var/lib/apt/lists/*

RUN adduser --force-badname --disabled-password --gecos '' --shell /bin/bash airsim_user && \
	echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
	adduser airsim_user sudo && \
	adduser airsim_user audio && \
	adduser airsim_user video

USER airsim_user
WORKDIR /home/airsim_user

# Change the following values to use a different AirSim binary
# Also change the AIRSIM_EXECUTABLE variable in docker-entrypoint.sh
ENV AIRSIM_BINARY_ZIP_URL=https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Blocks.zip
ENV AIRSIM_BINARY_ZIP_FILENAME=Blocks.zip

ENV SDL_VIDEODRIVER_VALUE=offscreen
ENV SDL_HINT_CUDA_DEVICE=0

RUN wget -c $AIRSIM_BINARY_ZIP_URL \
	&& unzip $AIRSIM_BINARY_ZIP_FILENAME \
	&& rm $AIRSIM_BINARY_ZIP_FILENAME

ADD ./app /home/airsim_user/app

WORKDIR /home/airsim_user
RUN mkdir -p /home/airsim_user/Documents/AirSim
ADD ./app/settings.json /home/airsim_user/Documents/AirSim
ADD ./docker/docker-entrypoint.sh /home/airsim_user/docker-entrypoint.sh

RUN sudo chown -R airsim_user /home/airsim_user

WORKDIR /home/airsim_user/app

RUN python3 -m pip install --user -r requirements.txt

ENTRYPOINT /home/airsim_user/docker-entrypoint.sh
