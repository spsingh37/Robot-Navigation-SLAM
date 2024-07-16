echo "[Setup] Apt Installing required packages..."
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential -y
echo "[Setup] Set up permission..."
sudo chown -R $USER:$USER .git
sudo chmod -R 775 .git
echo "[Setup] Updating submodules..."
cd lib && git submodule update --init
cd pico-sdk && git submodule update --init
# git checkout master
git pull
cd ../../
# install picotool
export PICO_SDK_PATH=$PWD/lib/pico-sdk
wget https://github.com/raspberrypi/picotool/archive/refs/tags/1.1.1.zip
unzip 1.1.1.zip
cd picotool-1.1.1
mkdir build && cd build
cmake ..
make
sudo make install
cd ../..
rm 1.1.1.zip
rm -rf picotool-1.1.1
echo "[Setup] Done!"
