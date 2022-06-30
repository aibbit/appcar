#!/bin/bash

while true
do
ifconfig eth1 192.168.1.100
echo "start appcat ..."
chmod +x /userdata/media/test/appcar/svr.out
/userdata/media/test/appcar/svr.out
echo "try to restart appcar ..."
sleep 10
done
