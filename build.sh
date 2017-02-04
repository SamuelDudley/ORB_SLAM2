echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../

if [ ! -f "./Vocabulary/ORBvoc.txt" ]; then

   echo "Uncompress text vocabulary..."
   cd Vocabulary
   tar -xf ORBvoc.txt.tar.gz
   cd ..

fi

if [ ! -f "./Vocabulary/ORBvoc.bin" ]; then

   echo "Converting text vocabulary to binary..."
   ./tools/bin_vocabulary
fi

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4

cd ..


