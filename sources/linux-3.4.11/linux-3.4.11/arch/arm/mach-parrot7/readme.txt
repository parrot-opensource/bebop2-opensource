Using clocks with the Parrot 7 Linux BSP :
==========================================

From within drivers you should use :

struct clk* clk_get(struct device*, char const*);
void clk_put(struct clk*);
int clk_enable(struct clk*);
void clk_disable(struct clk*);
unsigned long clk_get_rate(struct clk*);

Usage sample code:
------------------

static struct clk* myclk;

int myfunc(void)
{
    printk("%lu Hertz\n", clk_get_rate(myclk));
}

int init(struct platform_device* mydev)
{
    /* Get a reference to my clock. */
    myclk = clk_get(&mydev->dev, "core");

    /* activate clock. */
    clk_enable(myclk);
}

void fini(void)
{
    /* Turn clock off. */
    clk_disable(myclk);

    /* Do not forget this! Release clock reference. */
    clk_put(myclk);
}
