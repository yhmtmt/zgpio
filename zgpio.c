/*  zgpio.c  */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dmaengine.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>

#include <asm/uaccess.h>
#include <linux/amba/xilinx_dma.h>

#include "zgpio.h"

#ifndef __devinit 
#define __devinit
#define __devexit
#define __devinitdata
#endif

/* Standard module information*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR
("Yohei Matsumoto - Tokyo Univ. Marine Science and Technology <yhmtmt@kaiyodai.ac.jp>");
MODULE_DESCRIPTION
("zgpio - test driver module for led and switch of zedboard connected by AXI_GPIO ");

#define DRIVER_NAME "zgpio"

////////////////////////////////////////////////////////// Parameters
int tbuf_addr = 0x04;
int io2_addr = 0x08;
int tbuf2_addr = 0x0c;
unsigned int irq_gen_init = 0x80000000;
int irq_gen_addr = 0x11c;
unsigned int irq_en_init  = 0x00000000;
unsigned int irq_en_init2 = 0x00000000;
int irq_en_addr = 0x128;
unsigned int irq_st_mask  = 0x00000001;
unsigned int irq_st_mask2 = 0x00000002;
int irq_st_addr = 0x120;

int zgpio_major = 0; // major number (dynamically allocated in probe)
int zgpio_minor = 0; // minor number (zero fixed)
int zgpio_nr_devs = 1; // only one device node is supported.

module_param( tbuf_addr , int , S_IRUGO );
module_param( io2_addr , int , S_IRUGO );
module_param( tbuf2_addr , int , S_IRUGO );
module_param( irq_gen_init, int , S_IRUGO );
module_param( irq_gen_addr, int , S_IRUGO );
module_param( irq_en_init, int , S_IRUGO );
module_param( irq_en_init2, int , S_IRUGO );
module_param( irq_en_addr, int , S_IRUGO );
module_param( irq_st_mask, int , S_IRUGO );
module_param( irq_st_mask2, int , S_IRUGO );
module_param( irq_st_addr, int , S_IRUGO );

/////////////////////////////////////////////////////////// Driver's local data
struct zgpio_local {
  // for char device
  struct cdev cdev;
  struct semaphore sem; // for interface mutex
  // for gpio
  int irq;
  unsigned long mem_start;
  unsigned long mem_end;
  void __iomem *base_addr;
  struct fasync_struct * async_queue;
  int all_inputs;
  int all_inputs2;
  int dout_default;
  int dout_default2;
  int gpio_width;
  int gpio_width2;
  u32 tri_default;
  u32 tri_default2;
};

////////////////////////////////////////////////////////// file operation override
long zgpio_ioctl(struct file * filp, unsigned int cmd, unsigned long arg);
int zgpio_open(struct inode * inode, struct file * filp);
int zgpio_release(struct inode * inode , struct file * filp);
static int zgpio_fasync(int fd, struct file * filp, int mode);

////////////////////////////////////////////////////////// file operation object
struct file_operations zgpio_fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = zgpio_ioctl,
  .open = zgpio_open,
  .release = zgpio_release,
  .fasync = zgpio_fasync
};

/////////////////////////////////////////////////////////// fop implementation
long zgpio_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
  int err = 0;
  int retval = 0;
  unsigned int val;
  struct zgpio_local * lp = filp->private_data;

  if (_IOC_TYPE(cmd) != ZGPIO_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > ZGPIO_IOC_MAXNR) return -ENOTTY;
  
  if(_IOC_DIR(cmd) & _IOC_READ)
    err = !access_ok(VERIFY_WRITE, (void __user*) arg, _IOC_SIZE(cmd));
  else if(_IOC_DIR(cmd) & _IOC_WRITE)
    err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
  if(err) return -EFAULT;
  
  if(down_interruptible(&lp->sem))
    return -ERESTARTSYS;

  switch(cmd){
  case ZGPIO_IOCRESET:
    iowrite32(lp->tri_default, lp->base_addr + tbuf_addr);
    iowrite32(lp->tri_default2, lp->base_addr + tbuf2_addr);
    iowrite32(irq_gen_init, lp->base_addr + irq_gen_addr);
    iowrite32(irq_en_init | irq_en_init2, lp->base_addr + irq_en_addr);
    break;
  case ZGPIO_IOCSET:
    retval = __get_user(val, (unsigned int __user *) arg);
    iowrite32(val, lp->base_addr);
    break;
  case ZGPIO_IOCGET:
    val = ioread32(lp->base_addr);
    retval = __put_user(val, (unsigned int __user*) arg);
    break;
  case ZGPIO_IOCSET2:
    retval = __get_user(val, (unsigned int __user*) arg);
    iowrite32(val, lp->base_addr + io2_addr);
    break;
  case ZGPIO_IOCGET2:
    val = ioread32(lp->base_addr + io2_addr);
    retval = __put_user(val, (unsigned int __user*) arg);
    break;
  case ZGPIO_IOCSETGINT:
    retval = __get_user(val, (unsigned int __user*) arg);
    val = (val ? 0x80000000 : 0x00000000);
    iowrite32(val, lp->base_addr + irq_gen_addr);
    break;
  case ZGPIO_IOCSETINT:
    retval = __get_user(val, (unsigned int __user*) arg);
    val = (val ? 0x00000001 : 0x00000000);
    val |= ioread32(lp->base_addr + irq_en_addr) & ~irq_en_init;
    iowrite32(val, lp->base_addr + irq_en_addr);
    break;
  case ZGPIO_IOCSETINT2:
    retval = __get_user(val, (unsigned int __user*) arg);
    val = (val ? 0x00000002 : 0x00000000);
    val |= ioread32(lp->base_addr + irq_en_addr) & ~irq_en_init2;
    iowrite32(val, lp->base_addr + irq_en_addr);
    break;
  case ZGPIO_IOCSETTBUF:
    retval = __get_user(val, (unsigned int __user*) arg);
    iowrite32(val, lp->base_addr + tbuf_addr);
    break;
  case ZGPIO_IOCSETTBUF2:
    retval = __get_user(val, (unsigned int __user*) arg);
    iowrite32(val, lp->base_addr + tbuf2_addr);
    break;
  }

  up(&lp->sem);
  return retval;
}

int zgpio_open(struct inode * inode, struct file * filp)
{
  struct zgpio_local * lp;
 
  lp = container_of(inode->i_cdev, struct zgpio_local, cdev);
  filp->private_data = lp;
  return 0;
}

int zgpio_fasync(int fd, struct file * filp, int mode)
{
  struct zgpio_local * lp = (struct zgpio_local*) filp->private_data;
  return fasync_helper(fd, filp, mode, &lp->async_queue);
}

int zgpio_release(struct inode * inode , struct file * filp)
{
  struct zgpio_local * lp = (struct zgpio_local*) filp->private_data;
  if(!lp->async_queue){
    zgpio_fasync(-1, filp, 0);
  }

  return 0;
}

////////////////////////////////////////////////// fop end.

////////////////////////////////////////////////// platform driver functions
static irqreturn_t zgpio_irq(int irq, void *lp)
{
  unsigned int st;
  st = ioread32(((struct zgpio_local*)lp)->base_addr + irq_st_addr);
  if(((struct zgpio_local *) lp)->async_queue){
    kill_fasync(&((struct zgpio_local*) lp)->async_queue, SIGIO, POLL_IN);
  }
  
  iowrite32(st, ((struct zgpio_local *)lp)->base_addr + irq_st_addr);
  return IRQ_HANDLED;
}

static int zgpio_cdev_init(struct device * dev, struct zgpio_local * lp){
  dev_t devno;
  int rc = 0;
  
  if(zgpio_major){
    devno = MKDEV(zgpio_major, zgpio_minor);
    rc = register_chrdev_region(devno, zgpio_nr_devs, DRIVER_NAME);
  }else{
    rc = alloc_chrdev_region(&devno, zgpio_minor, zgpio_nr_devs, DRIVER_NAME);
    zgpio_major = MAJOR(devno);
  }
  
  dev_info(dev, "zgpio allocate cdev %d %d", zgpio_major, zgpio_minor);
  
  if(rc < 0){
    printk(KERN_WARNING "%s: can't get major %d\n", DRIVER_NAME, zgpio_major);
    return rc;
  }
  
  cdev_init(&lp->cdev, &zgpio_fops);
  lp->cdev.owner = THIS_MODULE;
  rc = cdev_add(&lp->cdev, devno, 1);
  if(rc){
    printk(KERN_NOTICE "Error %d adding %s%d", rc, DRIVER_NAME, 0);
    goto error;
  }

  return 0;

 error:
  unregister_chrdev_region(MKDEV(zgpio_major, zgpio_minor), zgpio_nr_devs);
  return -1;
}

static void zgpio_cdev_free(struct zgpio_local * lp){
  dev_t devno = MKDEV(zgpio_major, zgpio_minor);
  cdev_del(&lp->cdev);
  unregister_chrdev_region(devno, zgpio_nr_devs);
  free_irq(lp->irq, lp);
  release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
}

static int __devinit zgpio_probe(struct platform_device *pdev)
{
  struct resource *r_irq; /* Interrupt resources */
  struct resource *r_mem; /* IO mem resources */
  struct device_node * node;
  struct device *dev = &pdev->dev;  
  struct zgpio_local *lp = NULL;
  int all_inputs;
  int all_inputs2;
  int dout_default;
  int dout_default2;
  int gpio_width;
  int gpio_width2;
  u32 tri_default;
  u32 tri_default2;
  const __be32 * value;
  int rc = 0;

  /////////////////////////// Probing Device Tree ///////////////////////////////////////
  dev_info(dev, "Device Tree Probing\n");
  
  /* Get iospace for the device */
  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem) {
    dev_err(dev, "invalid address\n");
    return -ENODEV;
  }  

  /* Get IRQ for the device */
  r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
  if (!r_irq) {
    dev_info(dev, "no IRQ found\n");
  } 
 
  node = dev->of_node;
  value = of_get_property(node, 
			  "xlnx,all-inputs",
			  NULL);
  if(value)
    all_inputs = be32_to_cpup(value);
  else 
    all_inputs = 0;

  value = of_get_property(node, 
			  "xlnx,all-inputs-2",
			  NULL);
  if(value)
    all_inputs2 = be32_to_cpup(value);
  else 
    all_inputs2 = 0;

  value = of_get_property(node, 
			  "xlnx,dout-default",
			  NULL);
  if(value)
    dout_default = be32_to_cpup(value);
  else 
    dout_default = 0;

  value = of_get_property(node, 
			  "xlnx,dout-default-2",
			  NULL);
  if(value)
    dout_default2 = be32_to_cpup(value);
  else 
    dout_default2 = 0;

  value = of_get_property(node, 
			  "xlnx,gpio-width",
			  NULL);
  if(value)
    gpio_width = be32_to_cpup(value);
  else 
    gpio_width = 32;

  value = of_get_property(node, 
			  "xlnx,gpio2-width",
			  NULL);
  if(value)
    gpio_width2 = be32_to_cpup(value);
  else 
    gpio_width2 = 32;

  value = of_get_property(node, 
			  "xlnx,tri_default",
			  NULL);
  if(value)
    tri_default = be32_to_cpup(value);
  else 
    tri_default = 0;
 
  value = of_get_property(node, 
			  "xlnx,tri_default2",
			  NULL);
  if(value)
    tri_default2 = be32_to_cpup(value);
  else 
    tri_default2 = 0;
  
  ////////////////////////// allocating local data structure /////////////////////////////
  lp = (struct zgpio_local *) kzalloc(sizeof(struct zgpio_local), GFP_KERNEL);
  if (!lp) {
    dev_err(dev, "Cound not allocate zgpio device\n");
    return -ENOMEM;
  }

  lp->mem_start = r_mem->start;
  lp->mem_end = r_mem->end;

  lp->all_inputs = all_inputs;
  lp->all_inputs2 = all_inputs2;
  lp->dout_default = dout_default;
  lp->dout_default2 = dout_default2;
  lp->gpio_width = gpio_width;
  lp->gpio_width2 = gpio_width;
  lp->tri_default = tri_default;
  lp->tri_default = tri_default2;

  dev_set_drvdata(dev, lp);

  ////////////////////////// mapping gpio control memory ////////////////////////////////  
  
  if (!request_mem_region(lp->mem_start,
			  lp->mem_end - lp->mem_start + 1,
			  DRIVER_NAME)) {
    dev_err(dev, "Couldn't lock memory region at %p\n",
	    (void *)lp->mem_start);
    rc = -EBUSY;
    goto out_free_local;
  }
  
  lp->base_addr = ioremap(lp->mem_start, lp->mem_end - lp->mem_start + 1);

  if (!lp->base_addr) {
    dev_err(dev, "Could not allocate iomem\n");
    rc = -EIO;
    goto out_free_mem_region;
  }
  
  ////////////////////////// setting up gpio irq ///////////////////////////////////////
  if(r_irq){
    lp->irq = r_irq->start;
    
    rc = request_irq(lp->irq, &zgpio_irq, 0, DRIVER_NAME, lp);
    if (rc) {
      dev_err(dev, "Could not allocate interrupt %d.\n",
	      lp->irq);
      goto out_free_iomap;
    }
  }

  ///////////////////////// initializing gpio control register ////////////////////////

  // configuring axi gpio
  // determining io direction by applying tbuf_mask. 
  iowrite32(tri_default, lp->base_addr + tbuf_addr);
  iowrite32(tri_default2, lp->base_addr + tbuf2_addr);
  
  //global irq enabling
  iowrite32(irq_gen_init, lp->base_addr + irq_gen_addr);
  iowrite32(irq_en_init | irq_en_init2, lp->base_addr + irq_en_addr);
 
  /////////////////////////  initializing char device /////////////////////////////////
  if(zgpio_cdev_init(dev, lp) < 0)
    goto out_free_irq;

  sema_init(&lp->sem, 1);

  return 0;

  /* out_free_cdev:
     cdev_del(&lp->cdev);*/
 out_free_irq:
  free_irq(lp->irq, lp);
 out_free_iomap:
  iounmap(lp->base_addr);
 out_free_mem_region:
  release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
 out_free_local:
  kfree(lp);
  dev_set_drvdata(dev, NULL);
  return rc;
}

static int __devexit zgpio_remove(struct platform_device *pdev)
{
  struct device *dev = &pdev->dev;
  struct zgpio_local *lp = dev_get_drvdata(dev);

  /*
  dev_t devno = MKDEV(zgpio_major, zgpio_minor);
  cdev_del(&lp->cdev);
  unregister_chrdev_region(devno, zgpio_nr_devs);

  free_irq(lp->irq, lp);
  release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
  */

  zgpio_cdev_free(lp);
  kfree(lp);
  dev_set_drvdata(dev, NULL);
  return 0;
}

#ifdef CONFIG_OF
static struct of_device_id zgpio_of_match[] __devinitdata = {
  { .compatible = "xlnx,axi-gpio-1.01.b", },
  { .compatible = "xlnx,xps-gpio-1.00.a", },
  { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, zgpio_of_match);
#else
# define zgpio_of_match
#endif


static struct platform_driver zgpio_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table = zgpio_of_match,
  },
  .probe = zgpio_probe,
  .remove = zgpio_remove,
};

static int __init zgpio_init(void)
{
  printk(KERN_INFO "start zgpio. ver.0.00");
 
  return platform_driver_register(&zgpio_driver);
}


static void __exit zgpio_exit(void)
{
  platform_driver_unregister(&zgpio_driver);
  printk(KERN_INFO "end zgpio.\n");
}

module_init(zgpio_init);
module_exit(zgpio_exit);

