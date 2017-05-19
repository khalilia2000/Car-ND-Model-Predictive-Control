FROM ubuntu:16.04

RUN \
    apt-get update && \
    apt-get install wget build-essential -y && \
    wget https://cmake.org/files/v3.7/cmake-3.7.1.tar.gz && \
    tar xvf cmake-3.7.1.tar.gz && \
    cd cmake-3.7.1 && \
    ./configure && \
    make && \
    make install && \
    cd .. && rm -rf cmake-3.7.1 && \
        rm cmake-3.7.1.tar.gz && \
    apt-get -y clean all && \
    rm -rf /var/lib/apt/lists/*

# get sudo
RUN apt-get update
RUN apt-get install -y sudo

# Run Commands to install all the requierd packages
RUN apt-get update 
RUN sudo apt-get install -y \
    gcc \
    g++ \
    gfortran \
    libuv1-dev \
    libssl-dev \
    cppad \
    pkg-config \
    unzip \
    git \
    wget \
    cppad \
    python-matplotlib \ 
    python2.7-dev \
    dos2unix

# installing ipopt
ADD install_ipopt.sh .
RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
RUN dos2unix install_ipopt.sh
RUN bash install_ipopt.sh Ipopt-3.12.7
RUN rm -rf /var/lib/apt/lists/*

# adding the files and converting to unix
ADD install_uWS.sh .
RUN dos2unix install_uWS.sh
ADD clone_and_make.sh .
RUN dos2unix clone_and_make.sh

# cloning uWebSockets repository
RUN git clone https://github.com/uWebSockets/uWebSockets 

# install uWebSockets
RUN ./install_uWS.sh

# Make port 4567 available to the world outside this container
EXPOSE 4567