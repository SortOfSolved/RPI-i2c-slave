/*
 * Kernel space driver for BSC i2c slave mode on bcm2708.
 * 
 * Copyright (C) 2017 Rob Rescorla <rob@redaso.net>
 *
 * Inspired by i2c-bcm2708.c by Chris Boot & Frank Buss
 *
 * Which in turn was inspired by 
 * i2c-ocores.c, by Peter Korsgaard <jacmet@sunsite.dk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ktime.h>  //Maybe use getnstimeraw instead?!
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>

//includes we need for the sysfs instrumentation...
//A shout out to Grek Kroah-Hartman who wrote the object example stuff :-)
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#define DRV_NAME "pi_i2c_slave"

static int i2cslv_timezone,i2cslv_offset,i2cslv_sunday;

/* It's a device Keith, but not as we know it. */

/* Remember that as an i2c device, the degree of agency we have is limited.
 * A remote entity decides when the transaction starts, and unlike a server
 * We have a very tight window to respond in.  Although I2C supports 
 * bus-stalling The broadcom part doesn't.  If that's a complete show-stopper
 * then your best option is to rig up a gpio line to lock the i2c clock, I 
 * haven't considered what would happen if the pin was reasigned to gpio on 
 * the fly to GPIO and used to jam the bus, it's conceivably the kind of thing
 * that might depend on silicon revision... see later comment.
 *
 * So... we need to respond quickly, possibly within 30uS, and if you put all
 * your money on the idea of getting a usermode task to respond in that sort 
 * of time, good luck to you.
 *
 * So we cheat.
 *
 * We provide read-only access to a wall-clock time.
 * We implement a few bytes of RAM, which are available to userspace via /sys
 * There is no event interface to userspace in this driver.
 *
 * Regarding "Silicon revision":  It wouldn't surprise me if this hardware was
 * replaced with  something that works better.
 *
 * The i2c bus is on the other end of a pair of fifos.  We don't get per-byte
 * interrupts, we get fifo high/lo watermark interrupts.  
 *
 * On the input side we work around this by putting the interface in debug
 * mode, writing a dummy byte in, and putting it back to receive, so that the
 * next byte generates an event.
 *
 * On the output side, we flush the fifo after each write, because the write 
 * will have altered the address and so invalidated the output sequence.
 *
 * It gets rather worse though...
 *
 * We don't get any notification of transaction start or end, this means that 
 * we have to deduce which byte of the command we've just received.  (Although
 * there is a hardware feature for storing the first byte in the HCTRL 
 * register, this doesn't give us an interrupt, and so we have no notice to
 * update the output FIFO, worse still the HCTRL isn't syncronised to the 
 * input FIFO, so data bytes can be presented against HCTRL values for future
 * data.)
 *
 * And "brk" doesn't work properly, according to the datahsheet it's supposed
 * to flush both fifos.  It doesn't but flicking between brk and tx-enable
 * at the same time does cause a byte to be deleted from the output fifo... so
 * that's what I do. 
 * 
 * The datasheet is very patchy on this peripheral, and worryingly it isn't
 * mentioned on the RPi Foundation's datasheet for the compute module, (The
 * pinmux settings for it are blanked out.)  
 * 
 * If TX or RX aren't enabled, the device will still ACK even though the
 * write will be ignored and the read will return invalid data.
 *
 * You could be forgiven for thinking that it would be simpler to bit-bang 
 * the i2c function directly from the interrupt.
 * There are three factors that put this off.
 * 1) A fully compliant I2C implementation would require both clock edges to 
 *   be monitored, not just one.
 * 2) The other IRQ infrastructure of the kernel takes time to execute
 * 3) We'd need software safeguards to prevent loose connections (etc)
 *    causing crashes.
 *
 * If you have complete control of the i2c command set, you can work around
 * these problems more effificently than I have, and more reliably.
 * 
 * If you need better reliability, and if you can't craft the i2c requests
 * from the master to entertain you, then you'll need to bit-bang. 
 * 
 */
struct i2c_slv {
	void __iomem * base;//base offset of the peripheral
	int irq;
	bool txing,rxing,preset,last_val,gap_val;
	uint32_t soft_ctrl;//soft copy of the ctrl register
	spinlock_t lock;
	ktime_t last,gap,limit;
	//Last is the timestamp of the last event (valid if last_val is set)
	//Gap is the shortest gap so far.  (Only updated if preset is cleared.)
	//Limit is 80% of gap.
	uint8_t reg_addr;//address in the ram that we are pretending to be...
	uint8_t reg_addr_f;//and adjusted for the output fifo
	
};


static uint8_t i2c_ram[64];//The data storage.

static struct i2c_slv * hot_fudge;


//Borrowed from another driver...
static inline u32 bcm2708_rd(struct i2c_slv *s, unsigned reg)
{
	return readl(s->base + reg);
}

static inline void bcm2708_wr(struct i2c_slv *s, unsigned reg, u32 val)
{
	writel(val, s->base + reg);
}

/* BSC_SLV register offsets */
#define BSC_SLV_DATA		0x00
#define BSC_SLV_RSR		0x04 
#define BSC_SLV_ADDR		0x08 
#define BSC_SLV_CTRL		0x0c 
#define BSC_SLV_FLAG		0x10 
#define BSC_SLV_FLEV		0x14 
#define BSC_SLV_MASK		0x18 
#define BSC_SLV_RINT		0x1c 
#define BSC_SLV_MINT		0x20 
#define BSC_SLV_ICLR		0x24 
//according to 2835 peripheral datasheet as of 14/4/2017, DMA isn't supported 0x28 
#define BSC_SLV_TEST		0x2c 
//Not using with GPUSTAT 0x30
#define BSC_SLV_HCTRL 		0x42
//Debug registers aren't documented...
//DATA has status bits in upper bytes, but is read sensitive
static uint32_t rx_fifo_level(uint32_t dat)
{
return dat>>27;
}
static uint32_t tx_fifo_level(uint32_t dat)
{
return (dat>>22)&0x1f;
}
//modes.
//0, ctrl reg is 0
//1, running, ctrl reg is 
//2, flush one byte from tx
//3, stuff one byte into rx
static void bcm_i2c_do_ctrl(struct i2c_slv *s,uint32_t f)
{
uint32_t m;
m=s->soft_ctrl;
switch(f)
{
case 0:m=0;break;
case 1:m=0x305;break;//on, i2c_mode, tx&rx enabled
case 2:bcm2708_wr(s,BSC_SLV_CTRL,(m|0x80)&~0x100);break;
case 3:bcm2708_wr(s,BSC_SLV_CTRL,m|0x800);bcm2708_wr(s,BSC_SLV_TEST,0);break;
}
s->soft_ctrl=m;
bcm2708_wr(s,BSC_SLV_CTRL,m);
}

static ssize_t whole_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
int val;
if(!strcmp(attr->attr.name,"zone"))val=i2cslv_timezone;
else if(!strcmp(attr->attr.name,"delta"))val=i2cslv_offset;
else if(!strcmp(attr->attr.name,"sunday"))val=i2cslv_sunday;
else return -EINVAL;
return sprintf(buf, "%x\n", val);
}

static ssize_t whole_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
int ret,val;
ret=kstrtoint(buf,16,&val);
if(ret<0)return -EINVAL;
if(!strcmp(attr->attr.name,"zone"))i2cslv_timezone=val;
else if(!strcmp(attr->attr.name,"delta"))i2cslv_offset=val;
else if(!strcmp(attr->attr.name,"sunday"))i2cslv_sunday=val;
else return -EINVAL;
return count;
}




//FDT nodes return binary stuff, so this will as well!

static ssize_t ram_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
memcpy(&i2c_ram[0],buf,(count<64)?count:64);
return count;
}



static ssize_t ram_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
memcpy(buf,&i2c_ram[0],64);
return 64;
}

//deal in microseconds!
static ssize_t limit_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
int ret,val;
uint64_t t;
ret=kstrtoint(buf,16,&val);
if((val<0)||(val>1000000))return -EINVAL;
spin_lock(&hot_fudge->lock);
if(val) {t=val;t*=1000;hot_fudge->limit=t;hot_fudge->preset=true;}
else {hot_fudge->limit=0;hot_fudge->preset=false;}
spin_unlock(&hot_fudge->lock);
return count;
}


static ssize_t limit_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
uint64_t val;
val=hot_fudge->limit;
val/=1000;
return sprintf(buf, "%s as :%ld\n",hot_fudge->preset?"Set":"Calculated", val);
}
#if 0 
static struct class_attribute attrs[]={
__ATTR(limit,0664,limit_show,limit_store),
__ATTR(ram,0664,ram_show,ram_store),
__ATTR(zone,0664,whole_show,whole_store),
__ATTR(delta,0664,whole_show,whole_store),
__ATTR(sunday,0664,whole_show,whole_store),
__ATTR_NULL,
};
#endif 
static struct class i2c_slv_class = {
	.name="i2c_slv_data",
	.owner=THIS_MODULE,
#if 0 
	.class_attrs=attrs,//&attr_group,//or maybe attrs?
#endif 
};

static int instr_init(void)
{

int r;
int status;
	if((status=class_register(&i2c_slv_class))<0)return status;
return(r);
}
//With hind-sight, it would be neater if these nodes were grandchildren rather than direct children.
static void instr_exit(void)
{
class_unregister(&i2c_slv_class);

}
//INterrupts, according to 2835 datasheet i2c_m is on 53, i2c_s is 43...
//it would seem that the interrupts in bank 2 represent these in a linear but offset fashion
//so <2 16> is 48


//FIXME, fix the naming convention
static inline void bcm_i2c_slv_reset(struct i2c_slv *bi)
{
	bcm2708_wr(bi, BSC_SLV_RSR, 3);//clear errors
	bcm2708_wr(bi, BSC_SLV_ADDR, 0x34);//I2C Address, changed at request of camvys
/* Hardwiring the i2c address at the moment, long term there are two options.
 * 1) It isn't worth making it run-time configurable
 * 2) It somehow is */
        bcm_i2c_do_ctrl(bi,0);
        //Not at all convinced that flags can be written meaningfully
	bcm2708_wr(bi, BSC_SLV_FLEV, 0);//fifo levels not coherently explained...
	bcm2708_wr(bi, BSC_SLV_ICLR, 0xf);//clear all interrupts
	bcm2708_wr(bi, BSC_SLV_MASK, 0xf);//only interesed in TX/RX incidents.
}


static ktime_t moment;
static struct tm moment_dec;
static bool moment_valid;
#if 1 
static void ingest(struct i2c_slv * bi,bool field)
{
bool burn = true;//first byte in rx fifo is padding?
uint32_t expire,dat;
do {
	do {
		//w = bcm2708_rd(bi, BSC_SLV_CTRL);
		dat=bcm2708_rd(bi,BSC_SLV_DATA);
		if((!(dat&0xff))&&burn){burn=false;continue;}
		if(field)
			{
			field=false;
			bi->reg_addr=dat&0x3f;
			if((i2c_ram[0]&0x80)&&((dat&0x3f)<0x7))
				{
				moment=ktime_get_real();
				moment_valid=false;
				}
			}
		else {
			if(bi->reg_addr<7)moment_valid=true;
					//If client has written then
					//it has taken responsibility for accuracy of that data...
			i2c_ram[bi->reg_addr++]=dat;
			bi->reg_addr&=0x3f;
		}
	}while(rx_fifo_level(dat)>1);//FIFO field is as it was the instant read was called.
//So.. by this point input FIFO is empty, output buffer is invalidated...
//Things we have found out the hard way... if test mode is enabled received bytes will increase the FIFO count..
//BUT will place a zero in the FIFO 
//Conversely if tst is disabled nothing happens when we bash TEST!!!
//there is a very small race hazard here...
	bcm_i2c_do_ctrl(bi,3);//enable test
	dat=bcm2708_rd(bi,BSC_SLV_FLAG)<<16;
	}
while(rx_fifo_level(dat)>1);//vanishingly unlikely, 

//FIXME
//1) Over-flushes
//2) should be possible to salvage data in the output fifo		
for(expire=4;expire>0;expire--){ 
	bcm_i2c_do_ctrl(bi,2);//flush
	dat=bcm2708_rd(bi,BSC_SLV_FLAG);
	}while((tx_fifo_level(dat<<16)));
}

static void refill(struct i2c_slv *bi)
{
uint32_t dat;
do{
	if((bi->reg_addr_f<0x7)&&!moment_valid)
	{
		moment_valid=true;
		time64_to_tm(moment,0,&moment_dec);//FIXME need to introduce timezone and offset here.
		i2c_ram[0]&=0x80;i2c_ram[2]&=0x40;
		i2c_ram[0]|=((moment_dec.tm_sec%10)|(moment_dec.tm_sec/10)<<4);
		i2c_ram[1]=((moment_dec.tm_min%10)|(moment_dec.tm_min/10)<<4);
		if((i2c_ram[2]&0x40)&&(moment_dec.tm_hour>11)){i2c_ram[2]|=0x20;moment_dec.tm_hour-=12;}//Twelve hour mode, sort out AM/PM
		i2c_ram[2]|=((moment_dec.tm_hour%10)|(moment_dec.tm_hour/10)<<4);
		i2c_ram[3]=moment_dec.tm_wday;
		i2c_ram[4]=(moment_dec.tm_mday%10)|((moment_dec.tm_mday/10)<<4);
		if(moment_dec.tm_mon>9){i2c_ram[5]=0x10;moment_dec.tm_mon-=10;}
		else i2c_ram[5]=0;
		i2c_ram[5]|=moment_dec.tm_mon%10;
		moment_dec.tm_year%=100;
		i2c_ram[6]=(moment_dec.tm_year%10)|((moment_dec.tm_year/10)<<4);
	}
	bcm2708_wr(bi,BSC_SLV_DATA,i2c_ram[bi->reg_addr_f++]);bi->reg_addr_f&=0x3f;
	dat=bcm2708_rd(bi,BSC_SLV_FLAG);
}while((tx_fifo_level(dat<<16)<3));
}



//gap, the gap since the last byte
//limit, 0.8 of the shortest gap which involved a read-write transition.

static irqreturn_t bcm2708_slv_i2c_interrupt(int irq, void *dev_id)
{
	struct i2c_slv *bi = dev_id;
	bool update_gap=false;
	bool first=false;
	u32 s;
	ktime_t now,delta;
	now=ktime_get();
	spin_lock(&bi->lock);
        if(bi->last_val){
		delta=ktime_sub(now,bi->last);
		update_gap=!bi->preset;
		if(bi->gap_val||bi->preset)//gap valid or preset set.
	    		{
	    		if(ktime_compare(delta,bi->limit)>0)//this byte was too long after the predecessor.
				{
				first=true;//is a first byte
				//since limit is smaller than the shortest gap, it's possible that limit still needs recalculating.
				}
            		}
	}else
		{
		bi->last_val=true;
		first=true;
		}
	bi->last=now;
	//so by this point.
	//first is set if the byte has been received after such a long gap that it may be part of another transation.
	//update gap is set if delta should be considered to replace gap.

//logic... is this a send then this doesn't matter, however we update gap.
//if last has not been set, then this is a first byte
//if thw last interrupt was a read, then this is a first byte
//if gap has not been set *and* preset is not set, then it's not a first byte
//otherwise first status determined by comparison with limit.
	
	s = bcm2708_rd(bi, BSC_SLV_RINT);
	/* Here is the horror story: 
	 *
	 * We can't read the address that we were accessed with, *and* we 
	 * have no signal to indicate end of transaction.
	 * so when we receive data we have to guess which byte it is of 
         * the sent sequence from the master.
	 * ANNNNND Writing time sensitive data is problematic...
	 *
	 * Sooooo,
	 * we take the time of each interrupt...
         * Short intervals... not that we *absolutely* know what that means
	 * Indicate that the accesses are part of a single action, in an ideal
	 * world the gap will be 10-20 bit times.  (Oh wait, we don't know what
	 * a bit time is.
	 * Long intervals, we presume that the bus has been yielded.
	 *
	 * Our only hint we get is that if a different type of transaction has
	 * occured, then we know that it's the first byte.
	 *
	 * While the ktime_get looks like overkill, the data is either
	 * coming in so rarely that it doesn't matter, or so quickly that it's
	 * essential.
	 */
	switch(s&3){
	case 1://receive only.
		if(!bi->rxing){first=true;bi->rxing=true;bi->txing=false;}
		else update_gap=false;//we don't update gap unless the event type has changed
		break;
	case 2://transmit only.
		if(bi->txing)update_gap=false;
		bi->rxing=false;
		bi->txing=true;
	}
	if(s&1)ingest(bi,first);
	if(s&3)refill(bi);//need to refil if either fifo had an event.
	if(update_gap)
	   {
	   if(bi->gap_val)
		{if(ktime_compare(delta,bi->gap)<0)bi->gap=delta;else update_gap=false;}
	   else
		{bi->gap_val=true;bi->gap=delta;}
	   if(update_gap)bi->limit=(bi->gap/4)*5;
	   }
	spin_unlock(&bi->lock);
return IRQ_HANDLED;
}

#endif 

static int i2c_slave_probe(struct platform_device *pdev)
{
	struct resource *regs;
	int irq, index,t,w,pg, err = -ENOMEM;
	struct i2c_slv *bi;
        if(hot_fudge)
		{dev_err(&pdev->dev,"I2C slave probed too often!\n");return -1;}
//These values are populated from ... arch/arm/boot/dts/bcm2708_common.dtsi
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "could not get IO memory\n");
		return -ENXIO;
	}

	irq = platform_get_irq(pdev, 0);
        if(!irq)
	{
		return -ENXIO;
        //must free up memory and retrun fail.
	}
	bi = kzalloc(sizeof(*bi), GFP_KERNEL);
	if (!bi)
		goto no_mem;
	bi->irq=irq;
	bi->txing=bi->rxing=false;
        platform_set_drvdata(pdev, bi);
	instr_init();
        hot_fudge=bi;
	spin_lock_init(&bi->lock);
	bi->base = ioremap(regs->start, resource_size(regs));
	if (!bi->base) {
		dev_err(&pdev->dev, "could not remap memory\n");
		goto out_free_bi;
	}
	bi->irq = irq;

//This loop inspired by the spi code.
	err = request_irq(irq, bcm2708_slv_i2c_interrupt, IRQF_SHARED,
			dev_name(&pdev->dev), bi);
	if (err) {
		dev_err(&pdev->dev, "could not request IRQ: %d\n", err);
		goto out_iounmap;
	}
	bcm_i2c_slv_reset(bi);
	bcm_i2c_do_ctrl(bi,1);//switch on the interface
	bcm_i2c_do_ctrl(bi,3);//stuff the output
	return 0;
out_iounmap:
	iounmap(bi->base);
out_free_bi:
	kfree(bi);
no_mem:
	hot_fudge=NULL;
	return err;
}

static int i2c_slave_remove(struct platform_device *pdev)
{
	struct i2c_slv *bi = platform_get_drvdata(pdev);

	spin_lock(&bi->lock);

	bcm2708_wr(bi, BSC_SLV_MASK, 0x0);
	bcm_i2c_do_ctrl(bi,0);
	free_irq(bi->irq, bi);
	spin_unlock(&bi->lock);

	platform_set_drvdata(pdev, NULL);
	iounmap(bi->base);
	kfree(bi);

	return 0;
}

static const struct of_device_id i2c_slave_of_match[] = {
        { .compatible = "brcm,i2c-slave" },
        {},
};
MODULE_DEVICE_TABLE(of, i2c_slave_of_match);

static struct platform_driver bcm2708_i2c_slv = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = i2c_slave_of_match,
	},
	.probe		= i2c_slave_probe,
	.remove		= i2c_slave_remove,
};


static int __init bcm2708_i2c_slave_init(void)
{
hot_fudge=NULL;
	return platform_driver_register(&bcm2708_i2c_slv);
}

static void __exit bcm2708_i2c_slave_exit(void)
{
if(hot_fudge){instr_exit();hot_fudge=NULL;}
	platform_driver_unregister(&bcm2708_i2c_slv);
}

module_init(bcm2708_i2c_slave_init);
module_exit(bcm2708_i2c_slave_exit);



MODULE_DESCRIPTION("I2C Slave for Rapsberry Pi 3.  ");
MODULE_AUTHOR("Rob Rescorla <rob@redaso.net>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
