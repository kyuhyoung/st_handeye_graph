# in "st_handeye_graph"
cd ..
git clone https://bitbucket.org/koide3/st_handeye_eval.git
cd st_handeye_graph
docker build --force-rm --shm-size=64g -t st_handeye -f Dockerfile_wo_make .
#docker run --gpus all -it -v $PWD:/work/st_handeye_graph -v $PWD/../st_handeye_eval:/work/st_handeye_eval st_handeye:latest fish
bash docker_run_gui_mode_on_remote_server_from_local_server_with_option_1_container_name_and_tag_option_2_shared_dir.sh st_handeye:latest $PWD $PWD/../st_handeye_eval/
# in docker container
cd /root/st_handeye_graph
rm -rf build
mkdir build
cd build
#cmake .. -DCMAKE_MODULE_PATH=/work/st_handeye_graph/cmake -DVISP_DIR=~/visp-ws/visp-build/ -DCMAKE_BUILD_TYPE=Release
cmake .. -DVISP_DIR=~/visp-ws/visp-build/ -DCMAKE_BUILD_TYPE=Release
make -j4
cd ../scripts
