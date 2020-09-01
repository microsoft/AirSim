FROM nvidia/cudagl:9.0-devel

RUN apt-get update
RUN apt-get install \
	sudo \
	libglu1-mesa-dev \
	xdg-user-dirs \
	pulseaudio \
	sudo \
	x11-xserver-utils \
	unzip \
	wget \
	software-properties-common \
	-y --no-install-recommends

RUN apt-add-repository ppa:deadsnakes/ppa
RUN apt-get update
RUN apt-get install -y \
	python3.6 \
	python3-pip 

RUN python3.6 -m pip install --upgrade pip

RUN python3.6 -m pip install setuptools wheel

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

# Download and unzip the AirSim binary
RUN wget -c $AIRSIM_BINARY_ZIP_URL
RUN unzip $AIRSIM_BINARY_ZIP_FILENAME
RUN rm $AIRSIM_BINARY_ZIP_FILENAME

# Add the Python app to the image
ADD ./app /home/airsim_user/app

WORKDIR /home/airsim_user
RUN mkdir -p /home/airsim_user/Documents/AirSim
ADD ./app/settings.json /home/airsim_user/Documents/AirSim
ADD ./docker/docker-entrypoint.sh /home/airsim_user/docker-entrypoint.sh

RUN sudo chown -R airsim_user /home/airsim_user

WORKDIR /home/airsim_user/app

# Install Python requirements
RUN python3.6 -m pip install -r requirements.txt

ENTRYPOINT /home/airsim_user/docker-entrypoint.sh