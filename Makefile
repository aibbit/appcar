CC = arm-linux-gnueabihf-gcc
all:
	$(CC) *.c -lpthread -L. -lrv1126com -lm -o svr.out
clean:
	rm svr.out
send:
	scp ./svr.out root@192.168.1.100:/userdata/media/test/appcar/
