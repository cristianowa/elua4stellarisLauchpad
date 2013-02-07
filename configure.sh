#!bin/sh
cd ../
#install dependencies
apt-get install flex bison libgmp3-dev libmpfr-dev libncurses5-dev libmpc-dev autoconf texinfo build-essential libftdi-dev git python-yaml nasm scons python build-essential
#get compiler
wget https://launchpad.net/gcc-arm-embedded/4.6/4.6-2012-q4-update/+download/gcc-arm-none-eabi-4_6-2012q4-20121016.tar.bz2
tar -xjvf gcc-arm-none-eabi-4_6-2012q4-20121016.tar.bz2 
#install stellarisware
mkdir stellarisWare
cd stellarisWare
wget https://dl.dropbox.com/u/3154805/SW-LM3S-9453.exe 
unzip SW-EK-LM4F120XL-9453.exe
make
echo "you can find out stellaris code examples in the path ./stellarisware/boards/ek-lm4f120xl"
cd ..
#install flasher
git clone https://github.com/utzig/lm4tools.git
cd lm4tools/lm4flash
make
cp lm4flash gcc-arm-none-eabi-4_6-2012q4/bin/lm4flash
cd ../../
echo 'export PATH= $PATH:'$PWD/ gcc-arm-none-eabi-4_6-2012q4/bin/  >>$HOME/.bashrc
#try to compile elua to check if it is all right
cd elua4stellarisLauchpad
scons -f cross-lua.py
scons cpu=lm3s6965 toolchain=codesourcery prog
scons cpu=lm4f120h5qr toolchain=codesourcery prog
