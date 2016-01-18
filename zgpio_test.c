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
 
  if(argc < 3){
    printf("zgpio_test <device path> <operation> [<hex value>]\n");
    return 0;
  }

  fd = open(argv[1], O_RDWR);

  if(fd == -1) {
    printf("Failed to open %s", argv[1]);
    return 1;
  }

  if(strcmp(argv[2], "set") == 0){
    sscanf(argv[3], "%x", &val);
    ioctl(fd, ZGPIO_IOCSET, &val);
    printf("zgpio<=%08x\n", val);
  }else if(strcmp(argv[2], "get") == 0){
    ioctl(fd, ZGPIO_IOCGET, &val);
    printf("zgpio=>%08x\n", val);
  }else if(strcmp(argv[2], "sett") == 0){
    sscanf(argv[3], "%x", &val);
    ioctl(fd, ZGPIO_IOCSETTBUF, &val);
    printf("zgpio.tbuf<=%08x\n", val);    
  }else if(strcmp(argv[2], "gett") == 0){
    ioctl(fd, ZGPIO_IOCGETTBUF, &val);
    printf("zgpio.tbuf=>%08x\n", val);
  }else if(strcmp(argv[2], "set2") == 0){
    sscanf(argv[3], "%x", &val);
    ioctl(fd, ZGPIO_IOCSET2, &val);
    printf("zgpio<=%08x\n", val);
  }else if(strcmp(argv[2], "get2") == 0){
    ioctl(fd, ZGPIO_IOCGET2, &val);
    printf("zgpio2=>%08x\n", val);
  }else if(strcmp(argv[2], "sett2") == 0){
    sscanf(argv[3], "%x", &val);
    ioctl(fd, ZGPIO_IOCSETTBUF2, &val);
    printf("zgpio.tbuf2<=%08x\n", val);
  }else if(strcmp(argv[2], "gett2") == 0){
    ioctl(fd, ZGPIO_IOCGETTBUF2, &val);
    printf("zgpio.tbuf2=>%08x\n", val);
  }else if(strcmp(argv[2], "gint") == 0){
    val = (unsigned int) atoi(argv[3]);
    printf("gint<-%d\n", val ? 1 : 0);
    ioctl(fd, ZGPIO_IOCSETGINT, &val);
  }else if(strcmp(argv[2], "int") == 0){
    val = (unsigned int) atoi(argv[3]);
    printf("int<-%d\n", val ? 1 : 0);
    ioctl(fd, ZGPIO_IOCSETINT, &val);
  }else if(strcmp(argv[2], "int2") == 0){
    val = (unsigned int) atoi(argv[3]);
    printf("int2<-%d\n", val ? 1 : 0);
    ioctl(fd, ZGPIO_IOCSETINT2, &val);
  }else if(strcmp(argv[2], "sig") == 0){
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
    printf("Error: unknown operation %s.", argv[3]);
    return 1;
  }

  close(fd);

  return 0;
}
