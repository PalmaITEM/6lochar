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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/kfifo.h>

#include <linux/wait.h>
#include <linux/poll.h>

#include <linux/platform_device.h>
#include <linux/ieee802154.h>
#include <net/mac802154.h>
#include <net/cfg802154.h>


#define SUCCESS 0
#define DEVICE_NAME "char6lo"       /* Dev name as it appears in /proc/devices */
#define FIFO_LEN 12800    /* Max length (127+1*100, 100 frames + control byte) */


static int init_char(void);
static void cleanup_char(void);

static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static unsigned int device_poll(struct file *, poll_table *);

/*
 * Global Variables are declared as static, so are global within the file.
 * TODO: move these into the "private data" field of each device
 */

static struct file_operations fops = {
	.read = device_read,
	.write = device_write,
	.poll = device_poll,
	.open = device_open,
	.release = device_release
};

static int Major;           /* Major number assigned to our device driver */

static atomic_t sixlo_available = ATOMIC_INIT(1);	/* Is the device open?
                             * Used to prevent multiple access to the device */

/* These keep track of frames to be written */
static u8 missing_frame_bytes;
static struct sk_buff *w_skb;

/*
 * My priv structure
 * TODO do the same to char dev
 */
struct sixlo_phy {
	struct ieee802154_hw *hw;
	
	bool sleep;
	u8 page;
	u8 channel;
};

/* main pointer for driver */
static struct sixlo_phy *lphy;


static struct kfifo sixlo_fifo;   /* buffer to save created 6lowpan frames */
static wait_queue_head_t wait_rq; /* waiting queue for reading process */
static wait_queue_head_t wait_wq; /* waiting queue for writing process */
