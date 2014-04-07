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
  unsigned int val;
 
  fd = open("/dev/zgpio0", O_RDWR);
  if(argc < 2){
    printf("zgpio_test <operation> [<hex value>]\n");
    return 0;
  }

  if(strcmp(argv[1], "set") == 0){
    sscanf(argv[2], "%x", &val);
    ioctl(fd, ZGPIO_IOCSET, &val);
    printf("zgpio<=%08x\n", val);
  }else if(strcmp(argv[1], "get") == 0){
    ioctl(fd, ZGPIO_IOCGET, &val);
    printf("zgpio=>%08x\n", val);
  }else if(strcmp(argv[1], "set2") == 0){
    sscanf(argv[2], "%x", &val);
    ioctl(fd, ZGPIO_IOCSET2, &val);
    printf("zgpio<=%08x\n", &val);
  }else if(strcmp(argv[1], "get2") == 0){
    ioctl(fd, ZGPIO_IOCGET2, &val);
    printf("zgpio2=>%08x\n", val);
  }else if(strcmp(argv[1], "gint") == 0){
    val = (unsigned int) atoi(argv[2]);
    printf("gint<-%d\n", val ? 1 : 0);
    ioctl(fd, ZGPIO_IOCSETGINT, &val);
  }else if(strcmp(argv[1], "int") == 0){
    val = (unsigned int) atoi(argv[2]);
    printf("int<-%d\n", val ? 1 : 0);
    ioctl(fd, ZGPIO_IOCSETINT, &val);
  }else if(strcmp(argv[1], "int2") == 0){
    val = (unsigned int) atoi(argv[2]);
    printf("int2<-%d\n", val ? 1 : 0);
    ioctl(fd, ZGPIO_IOCSETINT2, &val);
  }else if(strcmp(argv[1], "sig") == 0){
    unsigned int pid = getpid();
    printf("Process id = %d\n", pid);
    fcntl(fd, F_SETOWN, pid);
    fcntl(fd, F_SETFL, FASYNC);
    /*
    sigset_t block;
    sigemptyset(&block);
    //    sigaddset(&block, SIGIO);
    struct sigaction sa = {
      .sa_handler = handler,
      .sa_flags = 0,
      .sa_mask = block
    };

    sigaction(SIGIO, &sa, 0);
    */
    signal(SIGIO, handler);
    signal(SIGINT, handler);

    while(!eflag);
    printf("Signal transfered.\n");

  }else{
    printf("Error: unknown operation %s.", argv[2]);
    return 1;
  }
  
  return 0;
}
