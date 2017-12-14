/* 
 *   Copyright (c) 2017 David Palma.
 *   All Rights Reserved.
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   <david.palma(at)ntnu(dot)no>
 */



/*
 *  This module intends to create a 6lowpan<->char interface
 *  NOTE: this version only stores the last packet received
 *
 *  FIXME the char device is created in an obsolete way
 *
 *  David Palma <david.palma(at)ntnu(dot)no>
 */

#include "6loch.h"

#define DRIVER_AUTHOR "David Palma <david.palma(at)ntnu(dot)no>"
#define DRIVER_DESC   "This module intends to create a 6lowpan<->char interface"


/*
 * This function is called when the module is loaded
 */
static int init_char(void)
{
	Major = register_chrdev(0, DEVICE_NAME, &fops);
	if (Major < 0) {
		printk(KERN_ALERT 
			"6loch registering char device failed with %d\n", Major);
		return Major;
	}
	
	printk(KERN_INFO "6loch assigned major number %d\n", Major);
	printk(KERN_INFO "6loch to use driver: 'mknod /dev/%s c %d 0'.\n",
		       	DEVICE_NAME, Major);
	
	return SUCCESS;
}

/*
 * This function is called when the module is unloaded
 */
static void cleanup_char(void)
{
	/*
	* Unregister the device
	*/
	unregister_chrdev(Major, DEVICE_NAME);
}

/*
 * Methods
 */

/*
 * We only allow 1 reader/writer at any given time
 */
static int device_open(struct inode *iniode, struct file *file)
{
	if (! atomic_dec_and_test(&sixlo_available)) {
		atomic_inc(&sixlo_available);
		return -EBUSY;
	}
	
	try_module_get(THIS_MODULE);
	
	return SUCCESS;
}

/* 
 * Called when a process closes the device file
 */
static int device_release(struct inode *inode, struct file *file)
{
	atomic_inc(&sixlo_available);	/* We're now ready for our next caller */
	/*
	 * Decrement the usage count, or else we'll never get rid of the module
	 */
	module_put(THIS_MODULE);
	
	return SUCCESS;
}

/*
 * Called when a process, which already opened the dev file, attempts to read
 * from it
 */
static ssize_t device_read(struct file *filp,
                            char *buffer,
                            size_t length,
                            loff_t * offset)
{
	int bytes_read = 0;
	int avail_len = 0;
	unsigned char *fifo_buffer;
	
	/*
	 * If no more data and NONBLOCKING return -EAGAIN, 
	 * to signify end of file or instead block!
	 */
	if (kfifo_is_empty(&sixlo_fifo)) {
		if (filp->f_flags&O_NONBLOCK) {
			/* Non-blocking mode */
			printk(KERN_ALERT "NONBLOCK mode");
			return -EAGAIN;
		} else {
			/* Blocking mode */
			printk(KERN_ALERT "BLOCK mode");
			if (wait_event_interruptible(wait_rq,
					!kfifo_is_empty(&sixlo_fifo)))
				return -ERESTARTSYS;
		}
	}	
	/* TODO: is it really necessary to alloc? or is it just a ref enough? */
	fifo_buffer = kmalloc(length, GFP_KERNEL);
	
	if ((avail_len = kfifo_out(&sixlo_fifo, fifo_buffer, length)) != length) {
		printk(KERN_INFO "Couldn't get all the requested bytes from fifo (ignore)\n");
	}
	
	/*
	 * Actually put the data into the users' buffer 
	 */
	while (length && avail_len) {
		/* 
		 * didn't use copy_to_user because
		 * flames... DocBook/kernel-hacking.tmpl 
		 */
		put_user(fifo_buffer[bytes_read], buffer++);
		
		length--;
		avail_len--;
		bytes_read++;
	}
	
	kfree(fifo_buffer);
	/*
	 * Most read functions return the number of bytes put into the buffer
	 */
	return bytes_read;
}

/*
 * Called when poll/select is used
 */
static unsigned int device_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &wait_rq,  wait);
	poll_wait(filp, &wait_wq, wait);
	if (!kfifo_is_empty(&sixlo_fifo))
		mask |= POLLIN | POLLRDNORM;	/* readable */

	mask |= POLLOUT | POLLWRNORM;	/* always writable */
	return mask;
}



/*
 * Called when a process writes to dev file
 */
static ssize_t
device_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	u8 w_len, written = 0;
	u8 w_frame_len = 0;

	while (len > 0) {
		if (!missing_frame_bytes) {
			/* use the first byte to indicate the frame length */
			w_frame_len = *buff++;
			len--;
			written++; /* not actually written but processed */
			
			/* validate length */
			if (!ieee802154_is_valid_psdu_len(w_frame_len)) {
				printk(KERN_ALERT 
					"6loch corrupted frame len received\n");
				return -EINVAL;
				/* w_frame_len = IEEE802154_MTU; */
			}

			w_skb = dev_alloc_skb(IEEE802154_MTU);
			if (!w_skb) {
				printk(KERN_ALERT 
					"6loch failed to allocate sk_buff\n");
				return -EINVAL;
			}
			
			missing_frame_bytes = w_frame_len;

		} /* else add to existing skb */

		if ( missing_frame_bytes > len ) {
			printk(KERN_INFO 
				"6loch write buffer has incomplete frame!\n");
			missing_frame_bytes -= len;
			w_len = len;
		} else {
			w_len = missing_frame_bytes;
			missing_frame_bytes = 0;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,13,0)
		memcpy(skb_put(w_skb, len), buff, w_len);
#else
		skb_put_data(w_skb, buff, w_len);
#endif

		if (!missing_frame_bytes) {
			ieee802154_rx_irqsafe(lphy->hw, w_skb, 0xcc);
		}

		written += w_len;
		buff += w_len;
		len -= w_len;
	}
	return written;
}


/* 6lowpan and main init */
static int sixlo_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	/* 
	 * This code was designed for one writer and one reader alone, therefore
	 * we use kfifo (circular buffer) and prevent any concurrency issues
	 */
	if (kfifo_avail(&sixlo_fifo) >= skb->len + 1) {
		kfifo_put(&sixlo_fifo, (u8) skb->len); /* add frame len info for reader */
		kfifo_in(&sixlo_fifo, skb->data, skb->len);
		wake_up_interruptible(&wait_rq);
	} else {
		/* the reader is not fast enough */ 
		printk(KERN_ALERT 
			"6loch xmit skbuf len larger the buffer, dropping!\n");
	}
	/* the xmit will always be successful, even if we drop the packet */ 
	ieee802154_xmit_complete(hw, skb, false);
	return SUCCESS;
}

/*
 * energy detection
 * (not implemented)
 */
static int sixlo_ed(struct ieee802154_hw *hw, u8 *level)
{
    printk(KERN_INFO "6loch ed\n");
    BUG_ON(!level);
    *level = 0xbe;
    return SUCCESS;
}

static int sixlo_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
    printk(KERN_INFO "6loch channel\n");
    return -EINVAL;
}

static int sixlo_start(struct ieee802154_hw *hw)
{
    struct sixlo_phy *lp = hw->priv;

    lp->sleep = false;

    return SUCCESS;
}

static void sixlo_stop(struct ieee802154_hw *hw)
{
    struct sixlo_phy *lp = hw->priv;

    lp->sleep = true;
}

/*
 * Not implemented/supported
 */
static int sixlo_set_hw_addr_filt(struct ieee802154_hw *hw,
			   struct ieee802154_hw_addr_filt *filt,
			   unsigned long changed)
{
    printk(KERN_INFO "6loch set hardware filter\n");

    /* Just for debug, not doing anything */
    if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
    	u16 addr = le16_to_cpu(filt->short_addr);
        printk(KERN_INFO 
    		 "6loch set_hw_addr_filt called for short addr: %X\n", addr);
    }
    
    if (changed & IEEE802154_AFILT_PANID_CHANGED) {
    	u16 pan = le16_to_cpu(filt->pan_id); 
        printk(KERN_INFO 
    		 "6loch set_hw_addr_filt called for pan id: %X\n", pan);
    }

    if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
        printk(KERN_INFO 
    		 "6loch set_hw_addr_filt called for IEEE addr: %llX\n",
		 filt->ieee_addr);
    }

    if (changed & IEEE802154_AFILT_PANC_CHANGED) {
    	printk(KERN_INFO 
		"6loch set_hw_addr_filt called for pan coordinator change: \n");
    	if (filt->pan_coord)
    	    printk(KERN_INFO "pan coordinator to true\n");
    	else
    	    printk(KERN_INFO "pan coordinator to false\n");
    }
    return SUCCESS;
}

static int sixlo_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
    printk(KERN_INFO "6loch set txpower\n");
    return -EINVAL;
}

/*
 * Listen Before Talk
 * (not implemented)
 */
static int sixlo_set_lbt(struct ieee802154_hw *hw, bool on)
{
    printk(KERN_INFO "6loch set lbt\n");
    return -EINVAL;
}

static int sixlo_set_cca_mode(struct ieee802154_hw *hw,
		       const struct wpan_phy_cca *cca)
{
    printk(KERN_INFO "6loch cca mode\n");
    return -EINVAL;
}

static int sixlo_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
    printk(KERN_INFO "6loch cca ed level\n");
    return -EINVAL;
}

static int sixlo_set_csma_params(struct ieee802154_hw *hw, u8 min_be,
                                     u8 max_be, u8 retries)
{
    printk(KERN_INFO
        "6loch set csma params -> min_be: %hhu, max_be: %hhu, retries: %hhu\n",
            min_be, max_be, retries);
    return SUCCESS;
}

static int sixlo_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
    printk(KERN_INFO "6loch frame retries -> retries: %hhd\n", retries);
    return SUCCESS;
}

static int sixlo_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
    printk(KERN_INFO "6loch promiscuous mode: \n");
    if (on)
        printk(KERN_INFO "set promiscuous mode true\n");
    else
        printk(KERN_INFO "set promiscuous mode false\n");

    return SUCCESS;
}


static const struct ieee802154_ops sixlo_ops = {
	.owner = THIS_MODULE,
	.xmit_async = sixlo_xmit,
	.ed = sixlo_ed,
	.set_channel = sixlo_channel,
	.start = sixlo_start,
	.stop = sixlo_stop,
	.set_hw_addr_filt = sixlo_set_hw_addr_filt,
	.set_txpower = sixlo_set_txpower,
	.set_lbt = sixlo_set_lbt,
	.set_cca_mode = sixlo_set_cca_mode,
	.set_cca_ed_level = sixlo_set_cca_ed_level,
	.set_csma_params = sixlo_set_csma_params,
	.set_frame_retries = sixlo_set_frame_retries,
	.set_promiscuous_mode = sixlo_set_promiscuous_mode,
};



static int sixlo_probe(struct platform_device *pdev)
{
	struct ieee802154_hw *hw;
	struct sixlo_phy *lp;
	int ret = 0;
	
	hw = ieee802154_alloc_hw(sizeof(*lp), &sixlo_ops);
	if (!hw)
	    return -ENOMEM;
	
	lp = hw->priv;
	lp->hw = hw;
	
	hw->flags = IEEE802154_HW_OMIT_CKSUM | IEEE802154_HW_CSMA_PARAMS |
	            IEEE802154_HW_FRAME_RETRIES | IEEE802154_HW_AFILT |
	            IEEE802154_HW_PROMISCUOUS;

	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER | WPAN_PHY_FLAG_CCA_ED_LEVEL |
	                WPAN_PHY_FLAG_CCA_MODE;
	
	hw->phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY) | 
	                BIT(NL802154_CCA_CARRIER) | BIT(NL802154_CCA_ENERGY_CARRIER);
	hw->phy->supported.cca_opts = BIT(NL802154_CCA_OPT_ENERGY_CARRIER_AND) |
	                BIT(NL802154_CCA_OPT_ENERGY_CARRIER_OR);
	
	hw->phy->cca.mode = NL802154_CCA_ENERGY;
	
	/* hw->flags = IEEE802154_HW_OMIT_CKSUM | IEEE802154_HW_PROMISCUOUS; */

	hw->phy->supported.channels[0] = 0x7FFF800;
	hw->phy->current_channel = 13;
	hw->parent = &pdev->dev;
	
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);
	
	ret = ieee802154_register_hw(lp->hw);
	if (ret)
	    ieee802154_free_hw(lp->hw);
	
	/* store our phy/dev/hw pointer */
	lphy = lp;
	
	return ret;
}

static int sixlo_remove(struct platform_device *pdev)
{
    ieee802154_unregister_hw(lphy->hw);
    ieee802154_free_hw(lphy->hw);

    return SUCCESS;
}

static struct platform_device *ieee802154stub_dev;

static struct platform_driver ieee802154stub_driver = {
        .probe = sixlo_probe,
        .remove = sixlo_remove,
        .driver = {
            .name = "ieee8021546lo",
        },
};

static int __init sixlo_init(void)
{
	int ret = 0;

	/* keeping track of written frames */
	missing_frame_bytes = 0;

	/* handle fifo and if successful create char dev and 15.4 hw */
	init_waitqueue_head(&wait_rq);
	init_waitqueue_head(&wait_wq);
	ret = kfifo_alloc(&sixlo_fifo, FIFO_LEN, GFP_KERNEL);
	if (ret)
	    return ret;
	
	/* handle char dev creation and if successful alloc 15.4 hw */
	ret = init_char();
	if (ret)
	    return ret;
	
	/* char dev init successful, continue */
	ieee802154stub_dev = platform_device_register_simple(
	                         "ieee8021546lo", -1, NULL, 0);
	return platform_driver_register(&ieee802154stub_driver);
}

static void __exit sixlo_cleanup(void)
{
    cleanup_char();
    platform_driver_unregister(&ieee802154stub_driver);
    platform_device_unregister(ieee802154stub_dev);
    /* no one will try to access this memory, free it! */
    kfifo_free(&sixlo_fifo);
}

module_init(sixlo_init);
module_exit(sixlo_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
