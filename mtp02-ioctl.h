#ifndef MTP02_IOCTL_H
#define MTP02_IOCTL_H

#include <linux/ioctl.h>

#define MAJOR_NUM 100

struct mtp02_settings
{
	int close_feed;
	int line_feed;
	int burn_time;
	int burn_count;
};

#define MTP02_FEED _IO(MAJOR_NUM,  0)
#define MTP02_GET_SETTINGS _IOR(MAJOR_NUM,  1, struct mtp02_settings)
#define MTP02_SET_SETTINGS _IOW(MAJOR_NUM,  2, struct mtp02_settings)

#endif
