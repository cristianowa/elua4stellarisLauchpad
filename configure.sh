#!/bin/sh
# IMPORTANT_1: execute this script with the following command: ". ./configure". This will retain the PATH variable change in current shell, 
# so you dont need to restart bash to read .bashrc again. 
# IMPORTANT_2: do NOT execute this script using sudo. This will create files with wrong ownership and permission problems.
cd ../

# Install dependencies
sudo apt-get install flex bison libgmp3-dev libmpfr-dev libncurses5-dev libmpc-dev autoconf texinfo build-essential libftdi-dev git python-yaml nasm scons python build-essential pkg-config libusb-1.0-0-dev libtool libusb-dev

# Get compiler
wget https://launchpad.net/gcc-arm-embedded/4.6/4.6-2012-q4-update/+download/gcc-arm-none-eabi-4_6-2012q4-20121016.tar.bz2
tar -xjvf gcc-arm-none-eabi-4_6-2012q4-20121016.tar.bz2

# Add compiler to PATH, if necessary
if [ "$(echo $PATH | grep gcc-arm-none-eabi-4_6-2012q4)" == "" ]; then
	export PATH=$PATH:$PWD/gcc-arm-none-eabi-4_6-2012q4/bin/
fi

# Add compiler to .bashrc, if necessary
if [ "$(cat ~/.bashrc | grep gcc-arm-none-eabi-4_6-2012q4)" == "" ]; then
	echo -e "\n"'export PATH=$PATH:'$PWD'/gcc-arm-none-eabi-4_6-2012q4/bin/' >> $HOME/.bashrc
fi
TARGET_BIN_DIR="$PWD"/gcc-arm-none-eabi-4_6-2012q4/bin/

# Install stellarisware
mkdir stellarisWare
cd stellarisWare
wget https://dl.dropbox.com/u/3154805/SW-LM3S-9453.exe
unzip SW-LM3S-9453.exe
make
echo "You can find out stellaris code examples in the path ./stellarisware/boards/ek-lm4f120xl"
cd ..

# Install flasher
git clone https://github.com/utzig/lm4tools.git
cd lm4tools/lm4flash
make all
cp lm4flash $TARGET_BIN_DIR
cd ../../

# Install debugger
git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd
cd openocd
git pull http://openocd.zylin.com/openocd refs/changes/22/922/10
./bootstrap
./configure --enable-maintainer-mode  --enable-ti-icdi
make
cd ..
cp -r ./openocd/tcl/* $TARGET_BIN_DIR
cp ./openocd/src/openocd $TARGET_BIN_DIR
cp elua4stellarisLauchpad/lm4f10xl.cfg $TARGET_BIN_DIR
echo 'cd '$TARGET_BIN_DIR' && openocd --file lm4f10xl.cfg  '>>$TARGET_BIN_DIR/openocd-lm4f
chmod +x $TARGET_BIN_DIR/openocd-lm4f

# Try to compile elua to check if it is all right
cd elua4stellarisLauchpad
scons -f cross-lua.py
scons cpu=lm3s6965 toolchain=codesourcery prog
scons cpu=lm4f120h5qr toolchain=codesourcery prog
