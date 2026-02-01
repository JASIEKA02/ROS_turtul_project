FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
	python3-pip \
	python3-colcon-common-extensions \
	git \
	nano \
	vim \
	&& rm -rf /var/lib/apt/lists/*


WORKDIR /ros2_ws
RUN mkdir -p src


RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
