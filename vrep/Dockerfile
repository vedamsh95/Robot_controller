FROM ubuntu:18.04
ARG VERSION=V-REP_PRO_EDU_V3_6_2_Ubuntu18_04

RUN apt-get update && apt-get install -y \
    wget xz-utils \
    libglib2.0-0 libgl1-mesa-glx libglu1-mesa-dev \
    xcb "libxcb*" libxkbcommon-x11-0 libx11-xcb-dev \
    libxrender-dev libxi6 libdbus-1-3 libfontconfig1 \
    libavcodec-dev libavformat-dev libswscale-dev \
    libqt5svg5 libsodium23 &&\
    rm -rf /var/lib/apt/lists/*
RUN mkdir /opt/vrep  && cd /opt/vrep &&\
    wget https://www.coppeliarobotics.com/files/$VERSION.tar.xz &&\
    tar -xf $VERSION.tar.xz --strip-components=1 &&\
    rm $VERSION.tar.xz

ENV QT_DEBUG_PLUGINS=1
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/vrep/

COPY runas /usr/local/bin/runas
ENTRYPOINT ["/usr/local/bin/runas"]
CMD ["/opt/vrep/vrep.sh"]
