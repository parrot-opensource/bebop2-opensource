#ifndef __P7MU_ADC_REGS_H
#define __P7MU_ADC_REGS_H

/*********************************/
/* P7MU SPI register and options */
/* Register address */
#define P7MU_SPI_CTRL_REG               0x00

/* Options */
#define P7MU_SPI_EN		(1 << 4)
#define P7MU_SPI_RESET		(1 << 5)
#define P7MU_SPI_DISABLE	(0 << 4)
#define P7MU_SPI_CPOL_CPHA_MASK	0x3
#define P7MU_SPI_MODE_0		0
#define P7MU_SPI_CPHA		0x1
#define P7MU_SPI_CPOL		0x2
#define P7MU_SPI_16MHZ		16000000
#define P7MU_SPI_8MHZ		8000000
#define P7MU_SPI_4MHZ		4000000
#define P7MU_SPI_2MHZ		2000000
#define P7MU_SPI_24MHZ		24000000
#define P7MU_SPI_12MHZ		12000000
#define P7MU_SPI_6MHZ		6000000
#define P7MU_SPI_3MHZ		3000000
#define P7MU_SPI_CLK_SEL_16MHZ	(0 << 2)
#define P7MU_SPI_CLK_SEL_8MHZ 	(1 << 2)
#define P7MU_SPI_CLK_SEL_4MHZ 	(2 << 2)
#define P7MU_SPI_CLK_SEL_2MHZ 	(3 << 2)
#define P7MU_SPI_CLK_SEL_24MHZ	(0 << 2)
#define P7MU_SPI_CLK_SEL_12MHZ	(1 << 2)
#define P7MU_SPI_CLK_SEL_6MHZ 	(2 << 2)
#define P7MU_SPI_CLK_SEL_3MHZ 	(3 << 2)


#endif /* __P7MU_ADC_REGS_H */

