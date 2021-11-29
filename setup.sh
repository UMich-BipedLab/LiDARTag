
ROOT_DIR=$PWD

git clone https://github.com/wjakob/tbb
mkdir -p $ROOT_DIR/tbb/build
cd $ROOT_DIR/tbb/build
cmake ..
cmake --build . --config Release -- -j 6
sudo cmake --build . --target install

cd $ROOT_DIR

git clone git://github.com/stevengj/nlopt
mkdir -p $ROOT_DIR/nlopt/build
cd $ROOT_DIR/nlopt/build
cmake ..
make
sudo make install

rm -rf $ROOT_DIR/tbb;
rm -rf $ROOT_DIR/nlopt;