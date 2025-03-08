CUR_DIR=`pwd`

echo "--- Downloading package"
mkdir -p tmp
curl https://s3.us-east-1.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2024_09_19.tgz --output tmp/TouchDriver_2024_09_19.tgz

echo "--- Extracting package"
cd tmp && tar xf TouchDriver_2024_09_19.tgz

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "--- Installing"
cd TouchDriver_2024_09_19
echo "--- Copying binarry files and shared libraries to /usr"
mkdir -p /usr/bin/TouchDriver_2024_09_19
cp bin/Touch* /usr/bin/TouchDriver_2024_09_19
cp usr/lib/libPhantomIOLib42.so /usr/lib/libPhantomIOLib42.so
cp usr/lib/libPhantomManagerLite.so /usr/lib/libPhantomManagerLite.so
echo "--- Copying udev rules to /etc/udev/rules.d"
cp rules.d/*.rules /etc/udev/rules.d/
echo "--- Reloading udev rules"
udevadm control --reload
udevadm trigger

echo "--- Create and configure shared directory for configuration files"
mkdir -p /usr/share/3DSystems/config

echo "--- Removing temporary files"
cd $CUR_DIR
rm -rf tmp

echo "--- Done"