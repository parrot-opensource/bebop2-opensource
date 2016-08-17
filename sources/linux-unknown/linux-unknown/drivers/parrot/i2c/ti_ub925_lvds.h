struct remap_i2c {
	unsigned int slave_id;
	unsigned int slave_alias;
};

/* Deserializer extra command */
struct dsr_i2c_cmd {
	u8 reg;
	u8 data;
};

struct ti_lvds_platform_data{
	/* nb of i2c slave of deser
	   allow to optimise speed if = 1 and premap is configured
	 */
	int nb_i2c_slave;
	/* i2c address to remap */
	struct remap_i2c premap;

	/* Deserializer command */
	struct dsr_i2c_cmd *cmd;

	/* Pixel Clock Edge Select */
	int clock_rising;

	/* Callback when deserializer detected */
	int (*deser_callback)(unsigned deser_addrgpio, const struct ti_lvds_platform_data **altconf);
};

#if 1
int
lvds_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags1,
	    const char *name, void *dev);
void lvds_free_irq(unsigned int irq, void *dev_id);
void lvds_enable_irq(unsigned int irq);
void lvds_disable_irq(unsigned int irq);
#else
static inline int lvds_request_irq(unsigned int irq, irq_handler_t handler,
				   unsigned long flags1, const char *name,
				   void *dev)
{
	return -EINVAL;
}

static inline void lvds_free_irq(unsigned int irq, void *dev_id)
{
}

static inline void lvds_enable_irq(unsigned int irq)
{
}

static inline void lvds_disable_irq(unsigned int irq)
{
}
#endif
