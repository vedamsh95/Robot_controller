FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y build-essential cmake inetutils-ping

COPY vrep/runas /usr/local/bin/

COPY ctrl ~/ctrl
WORKDIR ~/ctrl

RUN rm -rf "bin" && mkdir "bin" && cd "bin"
WORKDIR bin
RUN cmake ..
RUN make

ENTRYPOINT ["/usr/local/bin/runas"]
CMD ["./sdir_ctrl2020"]

