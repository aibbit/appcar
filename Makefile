CC = arm-linux-gnueabihf-gcc
all:
	$(CC) *.c -lpthread -L. -lrv1126com -lm -o svr.out
clean:
	rm svr.out
send:
	sudo ifconfig enp0s31f6 192.168.1.200
	scp ./svr.out root@192.168.1.100:/userdata/media/test/appcar/
