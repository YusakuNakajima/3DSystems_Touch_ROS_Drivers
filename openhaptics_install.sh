#!/bin/bash -e

CUR_DIR=`pwd`

echo "--- Downloading package"
mkdir -p tmp
curl https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz --output tmp/openhaptics_3.4-0-developer-edition-amd64.tar.gz

echo "--- Extracting package"
cd tmp && tar zxf openhaptics_3.4-0-developer-edition-amd64.tar.gz

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "--- Installing"
cd openhaptics_3.4-0-developer-edition-amd64

#copy files to opt folder
echo "--- Copying system files"
cp -R opt/* /opt
cp -R usr/lib/* /usr/lib
cp -R usr/include/* /usr/include

#Create symbolic links
echo ""
ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3.4
ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so

ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3.4
ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so

ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so.3.4
ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so

ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so.3.4
ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so

chmod -R 777 /opt/OpenHaptics

echo "--- DONE!"
echo ""

#Set path for OH_SDK_BASE
echo "--- Setting path for OH_SDK_BASE"
echo "export OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0" > /etc/profile.d/openhaptics.sh
echo "--- DONE!"
echo ""

echo "--- Installing dependencies"
apt install -y libncurses5-dev freeglut3 build-essential zlib1g-dev

echo "--- Removing temporary files"
cd $CUR_DIR
rm -rf tmp

echo "--- Done"

