CC = /home/test/dev/rv1126sdk/newbase/sdk/buildroot/output/firefly_rv1126_rv1109/host/bin/arm-linux-gnueabihf-gcc
all:
	$(CC) *.c -lpthread -L. -lrv1126com -lm -o svr.out
clean:
	rm svr.out
