    echo "--- Installing dependencies"
    apt-get install -y libqt5widgets5 libncurses5
    echo "--- Running TouchCheckup for connection test"
    cd /usr/bin/TouchDriver_2024_09_19
    ./TouchCheckup