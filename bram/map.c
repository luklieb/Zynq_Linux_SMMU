#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

typedef int bram_type;
#define print_type lld

static off_t phys_addr = 0x00A0000000;
static size_t len = 1<<12;


int main(int argc, char *argv[]) {
	printf("bram_type is %d bytes long\n", sizeof(bram_type));
	if (argc != 3) {
		printf("Usage: %s [r/w] number\n", argv[0]);
		return 0;
	}

	char mode = *argv[1];
	//bram_type number = atoll(argv[2]);
	bram_type number = strtol(argv[2], NULL, 16);
	printf("number as argument: %llx\n", number);
	// Truncate offset to a multiple of the page size, or mmap will fail.
	size_t pagesize = sysconf(_SC_PAGE_SIZE);
	off_t page_base = (phys_addr / pagesize) * pagesize;
	off_t page_offset = phys_addr - page_base;

	printf("pagesize: %zx, page_base: %zx, page_offset: %zx\n", pagesize, page_base, page_offset);

	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	bram_type *mem = (bram_type *) mmap(NULL, page_offset + len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
	if (mem == MAP_FAILED) {
		perror("Can't map memory");
		return -1;
	}

	printf("mmap successfull\n");	

	size_t read_number = 0;
	if (mode == 'w')
	{
		mem[0] = number;
		printf("wrote %llx, ", mem[0]);
		size_t i;
		for(i = 1; i < 5; ++i)
		{
			mem[i] = mem[i-1]*2;
			printf("%llx, ",mem[i]);
		}
		printf("\n Additional check...\n");

		for(i = 0; i < 5; ++i)
			printf("%llx, ", mem[i]);

		printf("\n");
		//printf("read: %d\n", mem[0]);
	}
	else if (mode == 'r')
	{
		printf("Reading from bram...\n");
		size_t i;
		for(i = 0; i < 5; ++i)
		{
			printf("%llx, ", (__int64_t)mem[i]);
		}	
		printf("\n");
	}
	else
	{
		printf("wrong mode\n");
	}

	return 0;
}
