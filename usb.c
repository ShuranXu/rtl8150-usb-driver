/*
 * 
 * Implement USB portion of rtl8139 Network Card driver
 * Use this as a template. Add code in areas matching string "CODE HERE".  
 * In this phase of the project we will be writing USB routines. 
 * Compile the driver as module. Use the makefile provided earlier
 * Make sure to unload the production module: 
 * # lsmod|grep rtl8150 
 * # rmmod rtl8150  
 * # lsmod 
 * Load the driver and run "ifconfig -a", You should see the MAC 
 * Address read from the device memory. 
 *
 */

#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>
#include <asm/uaccess.h>
#include "rtl8150.h"

#define DRV_NAME "LDDA_USB"  // Use it to change name of interface from eth
#define HAVE_NET_DEVICE_OPS

// rtl8150 Datasheet - Page 9, Vendor Specific Memory Read/Write Commands

#define RTL8150_REQT_READ       0xc0
#define RTL8150_REQT_WRITE      0x40

#define RTL8150_REQ_GET_REGS    0x05
#define RTL8150_REQ_SET_REGS    0x05

// Register offset in device memory  - rtl8150 Datasheet page 17 

#define	IDR	0x0120  	//  Device memory where MAC address is found

#define VENDOR_ID 			0x0bda
#define PRODUCT_ID 			0x8150

// Table of devices that work with this driver 
static struct usb_device_id rtl8150_table[] = {
	{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
	{0, }
};

/** 
  * This marks the usb_device_id table in the module image. This information 
  * loads the module on demand when the USBcard is inserted into  USB slot. 
  * It is part of Module auotload mechanism supported in Linux
  */

MODULE_DEVICE_TABLE(usb, rtl8150_table);


static struct usb_driver rtl8150_driver = {
	.name =		DRV_NAME,
	.id_table =	rtl8150_table,
	.probe =	rtl8150_probe,
	.disconnect =	rtl8150_disconnect
};

#ifdef HAVE_NET_DEVICE_OPS
static struct net_device_ops rtl8150_netdev_ops = {
        .ndo_open               = rtl8150_open,
        .ndo_stop               = rtl8150_close,
        .ndo_get_stats          = rtl8150_get_stats,
        .ndo_start_xmit         = rtl8150_start_xmit
};
#endif

/********* USB ROUTINES*************/
static int rtl8150_probe(struct usb_interface *intf,
                         const struct usb_device_id *id)
{
	struct net_device *dev;
	struct rtl8150 *priv;
	struct usb_device *udev;
	int rc;

	/* extract usb_device from the usb_interface structure */
	udev = interface_to_usbdev(intf);

	/**
	  * Linux Network Stack works with network device not the USB device. 
	  * We need to allocate an ethernet network device. alloc_etherdev() 
	  * allocates net device structure with memory allocated for 
	  * rtl8150 private structure. Return ENOMEM if failed
	  */
	dev = alloc_etherdev(sizeof(struct rtl8150));
	if(!dev){
		dev_err(&udev->dev, "Failed to allocate ethernet network device\n"); 
		return -1;
	}

	/** 
     * Set the device name to DRV_NAME instead of eth via memcpy 
	 *
     * What routine provides access to device's private structure from the 
     * net_device instance 
     */
	memcpy(dev->name, DRV_NAME, strlen(DRV_NAME));

    /* sysfs stuff. Sets up device link in /sys/class/net/interface_name */
    SET_NETDEV_DEV(dev, &intf->dev);

#ifdef HAVE_NET_DEVICE_OPS
        dev->netdev_ops = &rtl8150_netdev_ops;
#else
        dev->open = rtl8150_open;
        dev->stop = rtl8150_close;
        dev->hard_start_xmit = rtl8150_start_xmit;
        dev->get_stats = rtl8150_get_stats;
#endif

	/* Initialize Device private structure and initialize the spinlock */
	priv = netdev_priv(dev);

	spin_lock_init(&priv->lock);
	priv->netdev = dev;
	priv->udev = udev;
	priv->stats = dev->stats;

	/* Register netdevice. If fail goto out  */
	if(register_netdev(dev)){
		dev_err(&udev->dev, "Failed to register net device\n");
		goto freedev;
	}

    /**
      * We can stuff device private structure into USB Interface 
	  * structure using usb_set_intfdata. That can be retrieved later 
	  * using usb_get_intfdata, for example, in disconnect or other driver 
	  * functions
      */
	usb_set_intfdata(intf, priv);

	/**
      * Update net device struct with MAC and Broadcast address
      * RealTek8150 datasheet - page 17 states that IDR registers, IDR0-5, 
      * found at device memory location 0x0120 contains MAC Address of the
	  * the NIC device
	  * 
	  * Fill net device netdev->dev_addr[6] and netdev->broadcast[6] 
	  * arrays.  For broadcast address fill all octets with 0xff. 
	  * 
	  * In USB, you don't map device memory, instead you submit
	  * control urb to USB core when needs to read/write device register memory. 
	  * 
	  * The structure that sends the control request to a USB device has to 
	  * conform to USB specification (chapter 9) and is defined in 
	  *	include/linux/usb/ch8.h
	  *
	  * struct usb_ctrlrequest {
	  *	__u8 bRequestType;
	  *	__u8 bRequest;
	  *	__le16 mValue;
	  *	__le16 wIndex;
	  *	__le16 wLength;
	  * } __attribute__ ((packed));
	  * 
	  * Read IDR memory location in device memory by submitting 
	  * usb_control_msg() to USB core
	  * int usb_control_msg(struct usb_device *dev, unsigned int pipe,
	  * 	__u8 request, __u8 requesttype, __u16 value, 
      *		__u16 index, void *data, __u16 size, int timeout);
	  * where: 
	  *   - struct usb_device *dev: pointer to the target USB device
	  *   - unsigned int pipe: endpoint of the target USB device where
      *	this message will be sent. This value is created  by calling
	  *	usb_snd|usb_rcvctrlpipe
	  *   - __u8 request: Requets value for the control message 
	  *		- vendor specific (RTL8150_REQ_GET_REGS)
	  *   - __u8 requesttype: Request type 
      *      - vendor specific (RTL8150_REQT_READ)
	  *   - __u16 value: the USB message value for the control message
	  *	   driver uses it to specify the memory location in 
	  *	   device memory such as IDR register location.
	  *   - __u16 indx: Desired offset.  driver sets it to 0 
	  *	  	-  it means start of the requested memory location 
	  *   - void *data: A pointer to the data to send to the device if this
	  *	 is an OUT endpoint. If this is an IN endpoint, this is a
	  *	 pointer to where the data should be placed after being read
	  *	 from the device. 
	  *   - __u16 size: size of the buffer that is pointed to by the data
	  *	 paramter above. It is a number of bytes to transfer
	  *   - int timeout: The amount of time, in jiffies, that should be
	  *	waited before timing out. If this value is 0, the function
	  *	will wait forever for the message to complete. For USB compliant
	  *	host, device requests with a data stage must start to return data
	  *	in 500ms after the request.
	  *
	  *  Return Value: on success, it returns the number of bytes that
	  *  are transferred to or from the device, othervise negative number
	  *		
	  * Some parameters for usb_control_msg: request, requesttype, value, 
	  * indx,size map directly to the USB specification for how a USB 
	  * control message is defined.
	  *   	  
	  * usb_control_msg or usb_bulk_msg cannot be called from the 
	  * interrupt context. Also, this function cannot be cancelled by
	  * any other function, so be careful when using it; make sure that
	  * your driver disconnect function knows enough to wait for the call
	  * to complete before allowing itself to be unloaded from memory
	  * 
	  * Read six bytes into net_device structure member dev_addr from 
	  * device memory location IDR 
	  */ 

	memset(dev->broadcast, 0xff, 6);

	rc = usb_control_msg(priv->udev, usb_rcvctrlpipe(priv->udev, 0), RTL8150_REQ_GET_REGS,
	RTL8150_REQT_READ, IDR, 0, priv->netdev->dev_addr, sizeof(priv->netdev->dev_addr),
		500);

	if(rc < 0){
		dev_err(&udev->dev, "Failed to send the control message\n");
		goto out;
	}

	/* Length of Ethernet frame. It is a "hardware header length", number 
	* of octets that lead the transmitted packet before IP header, or 
	* other protocol information.  Value is 14 for Ethernet interfaces.
	*/

	dev->hard_header_len = 14;
	return 0;

out:
	usb_set_intfdata(intf, NULL);
	unregister_netdev(dev);
	free_netdev(dev);
	return -EIO;

freedev:
	free_netdev(dev);
	return -ENODEV;
}

static void read_bulk_callback(struct urb *urb)
{
	rtl8150_t *priv;
	struct net_device *dev;
	int res, pkt_len;
	int status;

	// Get access to priv struct and status of urb
	priv = urb->context;
	if(!priv)
		return;

	dev = priv->netdev;
	status = urb->status;
	switch (status) {
		case 0: //success
				break;
		case -ENOENT: /* the urb is in unlink state */
				return; 
		case -ETIME:
				printk("\n Reschedule it. Could be a problem with device\n");
				goto reschedule;
		default:
				printk("\nRx status %d\n", status);
				goto reschedule;
	}

	// we come here after receiving success status of urb
	res = urb->actual_length;   // Amount of data received in urb. Size of the packet
	pkt_len = res - 4; 		// discard 4-byte CRC

	/**
	* 1- Use skb_put to set skb->len and skb->tail to actual payload
	* 2- Set protocol field in skb
	* 3- Hand over the packet to protocol layer
	* 4- Increment rx_packet and rx_bytes
	*/	
	skb_put(priv->rx_skb, pkt_len);
	priv->rx_skb->protocol = eth_type_trans(priv->rx_skb, dev);
	netif_rx(priv->rx_skb);

	priv->stats.rx_packets ++;
	priv->stats.rx_bytes += pkt_len;

reschedule:     //submit again for next packet receive from the device

	/**
	  * Allocate sk_buff again and point priv->rx_skb to it
	  * populate bulk URB, Don't need to allocate urb, reuse it.
	  * submit urb. 
	  * CAUTION: code is running in atomic or /interrupt context
	  */
	priv->rx_skb = dev_alloc_skb(RTL8150_MTU + 2);
	skb_reserve(priv->rx_skb, 2);
	usb_fill_bulk_urb(priv->rx_urb, priv->udev, usb_rcvbulkpipe(priv->udev,1),
		priv->rx_skb->data, RTL8150_MTU, read_bulk_callback, priv);
	usb_submit_urb(priv->rx_urb);
	  
    return;
}

static void intr_callback(struct urb *urb)
{
	rtl8150_t *priv;
	__u8 *d;
	int status;

	// Get access to priv struct and status of urb
	priv = urb->context;
	if(!priv)
		return;
	status = urb->status;

	switch (status) {
		case 0:                 /* success */
				break;
		case -ECONNRESET:       /* unlink */
		case -ENOENT:
		case -ESHUTDOWN:
				return;
		default:
				printk("\n%s: intr status %d\n", priv->netdev->name, status);
				goto resubmit;
	}

	// We get here when status is set to success
	d = urb->transfer_buffer;
	if (d[0] & TSR_ERRORS) {
		priv->stats.tx_errors++;
		if (d[INT_TSR] & (TSR_ECOL | TSR_JBR))
				priv->stats.tx_aborted_errors++;
		if (d[INT_TSR] & TSR_LCOL)
				priv->stats.tx_window_errors++;
		if (d[INT_TSR] & TSR_LOSS_CRS)
				priv->stats.tx_carrier_errors++;
	}

	/* Report link status changes to the network stack */
	if ((d[INT_MSR] & MSR_LINK) == 0) {
		if (netif_carrier_ok(priv->netdev)) {
			netif_carrier_off(priv->netdev);
			printk("%s: LINK LOST\n", __func__);
		}
	} else {
		if (!netif_carrier_ok(priv->netdev)) {
			netif_carrier_on(priv->netdev);
			printk("%s: LINK CAME BACK\n", __func__);
		}
	}

resubmit:
	usb_submit_urb (urb, GFP_ATOMIC);
	return;
}


static void write_bulk_callback(struct urb *urb)
{
    printk(KERN_INFO "Entering %s\n", __FUNCTION__);

    rtl8150_t *priv;
	int status;

	// Get access to priv struct and status of urb
	priv = urb->context;
	if(!priv)
		return;
	status = urb->status;

	switch (status) {
		case 0:                 //success
			break;
		case -ENOENT:
			return; /*urb is in unlink state */
		case -ETIME:
			printk("\n Could be a problem with device\n");
			goto reenable;
		default:
			printk("\n%s: Tx status %d\n", priv->netdev->name, status);
			goto reenable;
	}

    // urb was transmitted successfully
	// Increment tx_packets and tx_bytes
	priv->stats.tx_packets ++;
	priv->stats.tx_bytes += urb->actual_length - 4; //discard 4-byte CRC 

	printk(KERN_INFO "\n%s: Queued Tx packet at %p size %u\n",
								priv->netdev->name, priv->tx_skb->data,
									priv->tx_skb->len);

	dev_kfree_skb_irq(priv->tx_skb);    //free skb, atomic context

    /**
      * Protocol layer should take care of error recovery. 
	  * We should just enable the queue so that protocol layer 
	  * continue to send skb_buff to us.
    */

reenable:	//enable the queue 
	netif_start_queue(priv->netdev);
    return;
}


//Net device routines 
static int rtl8150_open(struct net_device *netdev)
{
	printk(KERN_INFO "Entering %s\n", __FUNCTION__);

	int res;
	rtl8150_t *priv;

	/* Get address of private structure from net_device */
	priv = netdev_priv(netdev);
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev,0),
                RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE,IDR,0,
                        netdev->dev_addr, sizeof(netdev->dev_addr),500);

	/*
	* Driver sets up pipes to bulk IN (EP 1) endpoint for
	* receiving packets, bulk OUT (EP 2) for transmitting packets 
	* and to interrupt IN (EP 3) endpoint for receiving device errors 
	* and link status from the device:
	*
	* 1- Allocate memory for bulk IN, OUT and interrupt urb
	* 2- Allocate memory for sk_buff for bulk IN urb - receive
	* 3- Populate bulk IN urb and register call back function
	* 	NOTE: Make sure to allocate memory for intr_buff 
	* 4- Populate interrupt urb and register call back function
	* 5- Submit bulk IN and interrupt urb
	*
	* NOTE: Driver submits urb to bulk OUT EP from rtl8150_start_xmit
	* when the driver receives sk_buff from the protocol layer
	*/

	priv->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	priv->intr_urb = usb_alloc_urb(0, GFP_KERNEL);

	priv->rx_skb = dev_alloc_skb(RTL8150_MTU+2);
	skb_reserve(priv->rx_skb, 2);

	usb_fill_bulk_urb(priv->rx_urb, priv->udev, usb_rcvbulkpipe(priv->udev, 1),
		priv->rx_skb->data, RTL8150_MTU, read_bulk_callback, priv);

	priv->intr_buff = kmalloc(INTBUFSIZE, GFP_KERNEL);
	priv->intr_interval = 100; //poll interval for interrupt urb is 100 ms
	usb_fill_int_urb(priv->intr_urb, priv->udev, usb_rcvintpipe(priv->udev, 3),
		priv->intr_buff, INTBUFSIZE, intr_callback, priv, priv->intr_interval);

	res = usb_submit_urb(priv->rx_urb, GFP_KERNEL);
	if(res < 0) {
		dev_err(&priv->udev->dev, "Failed to submit the RX URB\n");
		return res;
	}
	
	res = usb_submit_urb(priv->intr_urb, GFP_KERNEL);
	if(res < 0) {
		dev_err(&priv->udev->dev, "Failed to submit the INTR URB\n");
		return res;
	}

	printk (KERN_INFO "\n Submit Rx and Intr urbs\n");

	// Initialize the hardware to make sure it is ready
	rtl8150_hardware_start(netdev);

	/* Notify the protocol layer so that it can start sending packet */
	netif_start_queue(netdev);

	printk(KERN_INFO "Exiting %s\n", __FUNCTION__);

	return res;
}


static void rtl8150_hardware_start(struct net_device *netdev)
{
	u8 data=0x10;
	u8 cr=0x0c;
	u8 tcr=0xd8;
	u8 rcr=0x9e;
	short tmp;
	int i = HZ;

	// Get address of device private structure
	rtl8150_t *priv;
	priv = netdev_priv(netdev);

	printk("Entering %s\n", __FUNCTION__);

	// Reset the chip. Make sure to wait for chip to reset
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev,0),
			RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE,CR,0,&data,1,500);

	// Confirm it that device has been reset successfully
	do {
		usb_control_msg(priv->udev, usb_rcvctrlpipe(priv->udev,0),
			RTL8150_REQ_GET_REGS, RTL8150_REQT_READ,CR,0,&data,1,500);
	} while ((data & 0x10) && --i);


	printk (KERN_INFO "\n DEVICE IS RESET SUCCESSFULLY\n");

	// Set RCR, TCR and CR registers to values in rcr,tcr and cr 
	// using usb_control_msg()
	
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
		RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, RCR, 0, &rcr, 1, 500);

	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
		RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, TCR, 0, &tcr, 1, 500);

	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
		RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, CR, 0, &cr, 1, 500);

	// Read CS Configuration Register (CSCR) register value in tmp, 
	// See Datasheet Page 29 , bit 3
	usb_control_msg(priv->udev, usb_rcvctrlpipe(priv->udev,0),
			RTL8150_REQ_GET_REGS, RTL8150_REQT_READ,CSCR,0,&tmp,1,500);

	// Check link status
	if (tmp & CSCR_LINK_STATUS)
		netif_carrier_on(netdev);
	else
		netif_carrier_off(netdev);

	printk (KERN_INFO "\n DEVICE CARRIER SET SUCCESSFULLY\n");
}

static int rtl8150_close(struct net_device *netdev)
{
	u8 cr;
	rtl8150_t *priv;
	/* Get address of private structure from net_device */
	priv = netdev_priv(netdev);

	printk(KERN_INFO "Entering %s\n", __FUNCTION__);

	//Notify protocol layer not to send any more packet to this interface
	netif_stop_queue(netdev);

	printk (KERN_INFO "\nrtl8150_close: shuting down the interface");

	// get the value of CR register into cr using usb_control_msg
	usb_control_msg(priv->udev, usb_rcvctrlpipe(priv->udev,0),
			RTL8150_REQ_GET_REGS, RTL8150_REQT_READ,CR,0,&cr,1,500);

	cr &= 0xf3;

	// Set the CR register to what is in cr using usb_control_msg
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
		RTL8150_REQ_SET_REGS, RTL8150_REQT_WRITE, CR, 0, &cr, 1, 500);

	/* Unlink all urbs */
	usb_unlink_urb(priv->rx_urb);
	usb_unlink_urb(priv->tx_urb);
	usb_unlink_urb(priv->intr_urb);
	usb_unlink_urb(priv->ctrl_urb);

	printk("Exiting %s\n", __FUNCTION__);
	return 0;
}

static int rtl8150_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
    int count,res;
	rtl8150_t *priv;

	/* Get address of private structure from net_device */
	priv = netdev_priv(netdev);

	printk(KERN_INFO "Entering %s\n", __FUNCTION__);

	// Make sure to stop the queue. Driver enable it when urb is completed
	netif_stop_queue(netdev);
	
	// Point tx_skb in private struct to skb received from protocol layer
	priv->tx_skb = skb;

	// Minimum packet length is 60 bytes
    count = (skb->len < 60) ? 60 : skb->len;

	/**
	  * Populate bulk OUT urb and register call back function
      * Submit urb to bulk OUT endpoint to USB core
	  * If submit urb fail, increment tx_errors count
	  * and renable the queue. 
	  * If urb is submitted successfully, just return 
      * Don't increment tx_packets, tx_bytes. It should be done 
	  * from call_back routine after successful status is recieved
	  */

	priv->tx_urb = usb_alloc_urb(0, GFP_KERNEL);

	usb_fill_bulk_urb(priv->tx_urb, priv->udev, usb_sndbulkpipe(priv->udev, 2),
	skb->data, skb->len, write_bulk_callback, priv);

	res = usb_submit_urb(priv->tx_urb, GFP_ATOMIC);

	if(res < 0){
		priv->stats.tx_errors ++;
		netif_start_queue(netdev);
	}	

    return res;
}

static struct net_device_stats* rtl8150_get_stats(struct net_device *dev)
{
	struct rtl8150 *priv = netdev_priv( dev );

	printk("dev_get_stats: Add code later\n");

	/**
	 * You cannot return NULL, make sure to return the address 
	 * of net_dev_stat that is in device private structure
	 */
	
	return &priv->stats;
}

/* USB disconnect routine - required else can't rmmod */
static void rtl8150_disconnect(struct usb_interface *intf)
{
	/* Get address of device private structure */
	struct rtl8150 *priv;
	priv = (struct rtl8150 *)usb_get_intfdata(intf);

	if (priv) {
		/**
		 * Unregister and free memory of net_device structure
		 * Call usb_set_intfdata(intf, NULL) to free memory	
		 */
		unregister_netdev(priv->netdev);
		free_netdev(priv->netdev);
		usb_set_intfdata(intf, NULL);
	}
}

/************* USB init and exit routines ***************/
static int __init usb_rtl8150_init(void)
{
	int ret;
	ret = usb_register(&rtl8150_driver);
	
	if(ret < 0) {
		pr_err("usb_register failed for the "__FILE__ "driver."
                    "Error number %d", ret);
        return -1;
	}

	return 0;
}

static void __exit usb_rtl8150_exit(void)
{
	/* deregister this driver with the USB subsystem */
    usb_deregister(&rtl8150_driver);
}

module_init(usb_rtl8150_init);
module_exit(usb_rtl8150_exit);

MODULE_AUTHOR("Shuran Xu");
MODULE_DESCRIPTION("USB Driver for Realtek rtl8150 USB Ethernet Wired Card");
MODULE_LICENSE("Dual BSD/GPL");
