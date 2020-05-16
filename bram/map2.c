#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
 
// Make the SDK console work in the debugger
#define printf(...) \
 fprintf(stdout, __VA_ARGS__); \
 fflush(stdout);
 
typedef long long int u64;
 
int main()
{
   unsigned int bram_size = 0x1000;
   off_t bram_pbase = 0xA0000000; // physical base address
   u64 *bram64_vptr;
   int fd;
 
   // Map the BRAM physical address into user space getting a virtual address for it
   if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
 
      bram64_vptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_pbase);
 
      // Write to the memory that was mapped, use devmem from the command line of Linux to verify it worked
      // it could be read back here also
 
      bram64_vptr[0] = 0xDEADBEEFFACEB00C;
      //bram64_vptr[0] =   0x0000000000000002;
      printf("read: %llx\n", bram64_vptr[0]);
      
      close(fd);
   }
 }
