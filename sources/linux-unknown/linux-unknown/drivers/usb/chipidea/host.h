#ifndef __DRIVERS_USB_CHIPIDEA_HOST_H
#define __DRIVERS_USB_CHIPIDEA_HOST_H

#ifdef CONFIG_USB_CHIPIDEA_HOST

int ci_hdrc_host_init(struct ci13xxx *ci);
void ci_hdrc_host_destroy(struct ci13xxx *ci);

#ifdef CONFIG_PM
void ci_hdrc_host_suspend(struct ci13xxx *ci);
void ci_hdrc_host_resume(struct ci13xxx *ci);
#endif

#else

struct ci13xxx;

static inline int ci_hdrc_host_init(struct ci13xxx *ci)
{
	return -ENXIO;
}

static inline void ci_hdrc_host_destroy(struct ci13xxx *ci)
{

}

#ifdef CONFIG_PM
static inline void ci_hdrc_host_suspend(struct ci13xxx *ci)
{
}

static inline void ci_hdrc_host_resume(struct ci13xxx *ci)
{
}

#endif


#endif

#endif /* __DRIVERS_USB_CHIPIDEA_HOST_H */
