// rtl8150 Datasheet - Page 9, Vendor Specific Memory Read/Write Commands

#define RTL8150_REQT_READ       0xc0
#define RTL8150_REQT_WRITE      0x40

#define RTL8150_REQ_GET_REGS    0x05
#define RTL8150_REQ_SET_REGS    0x05


// Register offsets in device memory  - rtl8150 Datasheet page 17

#define IDR     		0x0120  // MAC address is found here
#define CR                      0x012e
#define TCR                     0x012f
#define RCR                     0x0130
#define MSR                     0x0137	  // carrier lost or ok
#define MSR_LINK                (1<<2)   // bit 2 in MSR register - page 29
#define CSCR                    0x014C   // Link status is found here
#define CSCR_LINK_STATUS        (1 << 3) // bit 3 in CSCR register - Page 29
#define TSR                     0x0132

#define INTBUFSIZE              8

// Transmit status register errors 
#define TSR                     0x0132
#define TSR_ECOL                (1<<5)
#define TSR_LCOL                (1<<4)
#define TSR_LOSS_CRS            (1<<3)
#define TSR_JBR                 (1<<2)
#define TSR_ERRORS              (TSR_ECOL | TSR_LCOL | TSR_LOSS_CRS | TSR_JBR)

// Receive status register errors 
#define RSR_CRC                 (1<<2)
#define RSR_FAE                 (1<<1)
#define RSR_ERRORS              (RSR_CRC | RSR_FAE)

// Interrupt pipe data - for reporting errors and lost carrier

#define INT_TSR                 0x00
#define INT_RSR                 0x01
#define INT_MSR                 0x02
#define INT_WAKSR               0x03
#define INT_TXOK_CNT            0x04
#define INT_RXLOST_CNT          0x05
#define INT_CRERR_CNT           0x06
#define INT_COL_CNT             0x07

#define RTL8150_MTU             1540
#define RTL8150_TX_TIMEOUT      (HZ)

//Supported VenderID and DeviceID

#define VENDOR_ID       0x0bda          // Realtek Semiconductor Company
#define DEVICE_ID       0x8150          // RTL8150 USB Ethernet Controller

#define DRV_NAME "LDDA_USB"  // Use it to change name of interface from eth

// Device private structure 

struct rtl8150 {
        struct usb_device *udev;        // USB device
        struct net_device *netdev;      // Net device
        struct net_device_stats stats;  // Net device stats
        spinlock_t lock;
	// unsigned long flags;
	struct urb *rx_urb, *tx_urb, *intr_urb, *ctrl_urb;
	struct sk_buff *tx_skb, *rx_skb;
	int intr_interval;
	u8 *intr_buff;
};

typedef struct rtl8150 rtl8150_t;

// USB callback routines
static int rtl8150_probe(struct usb_interface *intf,
                           const struct usb_device_id *id);
static void rtl8150_disconnect(struct usb_interface *intf);
//static int rtl8150_suspend(struct usb_interface *intf, pm_message_t message);
//static int rtl8150_resume(struct usb_interface *intf);

static void read_bulk_callback(struct urb *urb);
static void write_bulk_callback(struct urb *urb);
static void intr_callback(struct urb *urb);

// Net Device specific routines 
static int rtl8150_open(struct net_device *netdev);
static int rtl8150_close(struct net_device *netdev);
static int rtl8150_start_xmit(struct sk_buff *skb, struct net_device *netdev);
static void rtl8150_hardware_start(struct net_device *netdev);
static struct net_device_stats* rtl8150_get_stats(struct net_device *netdev);

