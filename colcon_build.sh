colcon build --symlink-install --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
