all: test_suite

ifeq ($(REPT),1)
CFLAGS += -DREPT
endif


test_suite: size interf fault inv_range async latency

#timing.o: timing.c timing.h
#	gcc $<  ${CFLAGS} -c -o $@

interf: interf.c
	gcc $^ -o $@

fault: fault.c
	gcc $^ -o $@

size: size.c 
	gcc $^ ${CFLAGS} -O3 -lm -lrt -o $@

inv_range: inv_range.c
	gcc $^ ${CFLAGS} -lrt -o $@

async: async.c
	gcc $^ ${CFLAGS} -lrt -O3 -o $@

latency: latency.c
	gcc $^ ${CFLAGS} -lrt -O3 -o $@

clean:
	rm -rf size interf fault inv_range async latency *.o
