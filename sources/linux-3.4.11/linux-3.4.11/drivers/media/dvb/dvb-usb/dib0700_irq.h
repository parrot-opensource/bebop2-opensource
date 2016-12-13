#if defined(CONFIG_DVB_USB_DIB0700) || defined(CONFIG_DVB_USB_DIB0700_MODULE)

extern int
dib_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags1,
		const char *name, void *dev);
extern void dib_free_irq(unsigned int irq, void *dev_id);
extern void dib_enable_irq(unsigned int irq);
extern void dib_disable_irq(unsigned int irq);

#else
static inline int dib_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags1, const char *name, void *dev)
{
	printk(KERN_ERR "Can't request dib irq, dib0700 is module");
	return -EINVAL;
}
static inline void dib_free_irq(unsigned int irq, void *dev_id)
{
}
static inline void dib_enable_irq(unsigned int irq)
{
}
static inline void dib_disable_irq(unsigned int irq)
{
}

#endif
