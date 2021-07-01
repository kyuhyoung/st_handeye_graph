FROM vispci/vispci:ubuntu-20.04
WORKDIR /home

RUN apt-get update
RUN apt-get install -y cmake git build-essential

RUN git clone https://github.com/libigl/eigen.git
RUN cd eigen && mkdir build && cd build && cmake .. && make -j4 && make install

RUN apt-get update && apt-get install -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 && rm -rf /var/lib/apt/lists/*
RUN mkdir /code && cd /code && git clone https://github.com/RainerKuemmerle/g2o.git && cd ./g2o && mkdir build && cd build && cmake ../ && make -j4 && make install -j4
RUN ldconfig

ADD ./* $HOME/st_handeye_graph/
WORKDIR $HOME/st_handeye_graph/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
CMD bash
