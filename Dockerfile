FROM vispci/vispci:ubuntu-20.04
WORKDIR /home

RUN apt-get update
RUN apt-get install -y cmake git build-essential

RUN git clone https://github.com/libigl/eigen.git
RUN cd eigen && mkdir build && cd build && cmake .. && make -j4 && make install

RUN apt-get update && apt-get install -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 && rm -rf /var/lib/apt/lists/*
RUN find / -name "VISP*.cmake"
RUN mkdir /code && cd /code && git clone -b 20170730_git https://github.com/RainerKuemmerle/g2o.git && cd ./g2o && mkdir build && cd build && cmake ../ && make -j4 && make install -j4
RUN ldconfig

RUN find / -name "Find*.cmake"
ADD ./* $HOME/st_handeye_graph/
ADD ./src $HOME/st_handeye_graph/src
ADD ./include $HOME/st_handeye_graph/include
ADD ./scripts $HOME/st_handeye_graph/scripts
ADD ./cmake $HOME/st_handeye_graph/cmake
WORKDIR $HOME/st_handeye_graph/build
RUN echo $CMAKE_MODULE_PATH
RUN find / -name "g2o*.cmake"
RUN pwd
RUN ls -altrsh
RUN echo ${OpenCV_INCLUDE_DIRS}
RUN find / -name "libg2o*"
RUN ls -altrsh
#RUN cmake .. -DG2O_LIBRARY_DIRS=/usr/local/lib/  -DVISP_DIR=/root/visp-ws/visp-build/ -DG2O_DIR=/usr/local/lib/cmake/g2o/ -DCMAKE_BUILD_TYPE=Release && make -j4
#RUN cmake .. -DG2O_DIR=/usr/local/lib/cmake/g2o/ -DCMAKE_MODULE_PATH=/root/st_handeye_graph/cmake -DVISP_DIR=/root/visp-ws/visp-build/ -DCMAKE_BUILD_TYPE=Release && make -j4
RUN cmake .. -DCMAKE_MODULE_PATH=/root/st_handeye_graph/cmake -DVISP_DIR=/root/visp-ws/visp-build/ -DCMAKE_BUILD_TYPE=Release && make -j4
#RUN cmake .. -DVISP_DIR=/root/visp-ws/visp-build/ -DCMAKE_BUILD_TYPE=Release && make -j4
#RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
RUN apt-get update && apt-get install -y fish
CMD bash
