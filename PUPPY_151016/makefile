CFLAGS = -g -D_DEBUG_
CC = gcc
GENERAL_OBG = main.o dispatch_cmd.o  motorThread.o closeloop.o ad_da.o \
							laserThread.o scanThread.o new_dds_IO.o 

all: sever
.PHONY : all
sever : $(GENERAL_OBG)
	$(CC) $(CFLAGS) -o sever -lm -lpthread $(GENERAL_OBG) 

main.o : afm_comm.h work_thread.h hardware.h 
dispatch_cmd.o : work_thread.h hardware.h
motorThread.o : work_thread.h hardware.h
closeloop.o : hardware.h
ad_da.o : hardware.h
laserThread.o : work_thread.h hardware.h
scanThread.o : work_thread.h hardware.h closeloop.h
new_dds_IO.o : afm_comm.h hardware.h work_thread.h

.PHONY : clean
clean : 
	rm $(GENERAL_OBG)
