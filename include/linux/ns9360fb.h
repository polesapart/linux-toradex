#ifndef __NS9360FB_H__
#define __NS9360FB_H__

struct ns9360fb_pdata {
	unsigned height;
	unsigned width;
	unsigned clock;

	u32 timing[4];
	u32 control;
};

#endif
