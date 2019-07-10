#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <fcntl.h>
#include <signal.h>
#include "zgpio.h"

#define DATA_ALIGN 3
#define DATA_SIZE (1 << DATA_ALIGN)
struct thdata{
  int * buf;
  int fd;
  int res;
};

volatile sig_atomic_t eflag = 0;
void handler(int signo){
  eflag = 1;
  printf("signal SIGIO received.\n");
}

int main(int argc, char ** argv){
  int fd;
  unsigned long flag_set, flag_get;
  int ipin;
  int pmask;
  unsigned int val;
 
  if(argc != 3){
    printf("zgpio_get <device path> <pin number>\n");
    return 0;
  }

  ipin = atoi(argv[2]);
  if(ipin < 0 || ipin >= 64){
    printf("<pin number> should be in 0 .. 63.\n");
    return 0;
  }
  if (ipin < 32){
    flag_set = ZGPIO_IOCSET;
    flag_get = ZGPIO_IOCGET;
  }else{
    flag_set = ZGPIO_IOCSET2;
    flag_get = ZGPIO_IOCGET2;    
  }
  
  ipin = ipin % 32;
  pmask = 0x00000001 << ipin;
  
  fd = open(argv[1], O_RDWR);
  if(fd == -1) {
    printf("Failed to open %s", argv[1]);
    return 1;
  }

  ioctl(fd, flag_get, &val);
  if(val & pmask)
    printf("1\n");
  else
    printf("0\n");
  
  close(fd);

  return 0;
}
