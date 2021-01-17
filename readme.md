# OAK-D Kimera Experiment

This is an experiment in getting Kimera to work with an OAK-D.

At the moment all SLAM processing is done on the host, while we wait for custom
nodes to be added to OAK-D's Gen2 pipeline.

TODO: More info

## Build

1. Download ORB_SLAM3 submodule depthai-core and it's submodules:
    ```
    git submodule init
    cd depthai-core
    git submodule update --init --recursive
    ```
2. Install Kimera, following [their install
   instructions](https://github.com/MIT-SPARK/Kimera-VIO/blob/master/docs/kimera_vio_install.md).
   I had a lot of difficulty with this, so make sure:
   - gtsam is checked out to `ee069286b447ff58b809423cc77c777a02abdfe5`
   - if you accidentally installed a gtsam version other than the one above you
     may have an incompatible shared library in `/usr/local/lib/libgtsam*`.
     Kimera needs `libgtsam.so.4.0.2`, so you may need to change the symbolic
     link of `libgtsam.so.4` to point to this rather than some other version.
     Do this for `libgtsam_unstable*` as well.
   - if (like me, for some ridiculous reason) have multiple OpenCV library
     versions installed make sure the correct version is specified in the
     CMakeLists.txt for DBoW2 and Kimera-VIO
   - fix Kimera-VIO visualizer, which seems to be a little broken for me, fix
     is [here](https://github.com/MIT-SPARK/Kimera-VIO/issues/134)

   When making Kimera-VIO itself you can use the following to also install the
   library, for use in this repo.
   ```
   sudo make -j $(nproc) install 
   ```
3. Download the vocab file and unzip into `vocabulary` (link to file)[https://www.dropbox.com/s/lyo0qgbdxn6eg6o/ORBvoc.zip?dl=0]
3. Build the project:
    ```
    mkdir build && cd build
    cmake ..
    make -j $(nproc)
    ```
5. From the root of this repo run the experiment:
    ```
    ./oakd_kimera.bash
    ```
