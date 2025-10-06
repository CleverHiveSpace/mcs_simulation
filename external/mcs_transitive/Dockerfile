FROM ros:humble-ros-base

RUN apt-get update
RUN apt-get install -y build-essential curl git lsb-release gnupg

COPY entrypoint.sh .

CMD ["./entrypoint.sh"]