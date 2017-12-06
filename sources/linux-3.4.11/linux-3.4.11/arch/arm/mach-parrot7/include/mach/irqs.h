/*
 *  linux/arch/arm/mach-parrot7/include/mach/irqs.h
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  28-Oct-2010
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _ARCH_PARROT7_IRQS_H
#define _ARCH_PARROT7_IRQS_H

/* Private Peripherals Interrupts. */
#define P7_LOCALTIMER_IRQ	29
#define P7_LOCALWDOG_IRQ	30


/* Shared Peripherals Interrupts. */
#define P7_SPIRQ(_irq)      (32 + (_irq))

/* GPU IRQ for MALI200: only on P7 R1 & R2 */
#define P7_GPU_IRQ0         P7_SPIRQ(0)
#define P7_GPU_IRQ1         P7_SPIRQ(1)
#define P7_GPU_IRQ2         P7_SPIRQ(2)
/* GPU IRQ for MALI400: only on P7 R3 */
#define P7_GPU_PP0_IRQ      P7_SPIRQ(85)
#define P7_GPU_PPMMU0_IRQ   P7_SPIRQ(86)
#define P7_GPU_PP1_IRQ      P7_SPIRQ(87)
#define P7_GPU_PPMMU1_IRQ   P7_SPIRQ(88)
#define P7_GPU_PP2_IRQ      P7_SPIRQ(89)
#define P7_GPU_PPMMU2_IRQ   P7_SPIRQ(90)
#define P7_GPU_PP3_IRQ      P7_SPIRQ(91)
#define P7_GPU_PPMMU3_IRQ   P7_SPIRQ(92)
#define P7_GPU_GP_IRQ       P7_SPIRQ(93)
#define P7_GPU_GPMMU_IRQ    P7_SPIRQ(94)

#define P7_NAND_IRQ         P7_SPIRQ(3)
#define P7_SDIO0_IRQ	    P7_SPIRQ(11)
#define P7_SDIO1_IRQ	    P7_SPIRQ(12)
#define P7_SDIO2_IRQ	    P7_SPIRQ(13)

/* SPI IRQ: 4 to 7 */
#define P7_SPI0_IRQ         P7_SPIRQ(4)
#define P7_SPI1_IRQ         P7_SPIRQ(5)
#define P7_SPI2_IRQ         P7_SPIRQ(6)
#define P7_SPI3_IRQ         P7_SPIRQ(7)

/* USB IRQ: 8 to 10 */
#define P7_USB0_IRQ	    P7_SPIRQ(8)
#define P7_USB1_IRQ	    P7_SPIRQ(9)

/* DMAC IRQs on P7 R1: 15 to 21 */
#define P7_R1_DMA_ABORT_IRQ    P7_SPIRQ(15)
#define P7_R1_DMA0_IRQ         P7_SPIRQ(16)
#define P7_R1_DMA1_IRQ         P7_SPIRQ(17)
#define P7_R1_DMA2_IRQ         P7_SPIRQ(18)
#define P7_R1_DMA3_IRQ         P7_SPIRQ(19)
#define P7_R1_DMA4_IRQ         P7_SPIRQ(20)
#define P7_R1_DMA5_IRQ         P7_SPIRQ(21)

/*Ethernet IRQs: 15 to 17 */
#define P7_ETH0_IRQ         P7_SPIRQ(15)
#define P7_ETH1_IRQ         P7_SPIRQ(16)
#define P7_ETH2_IRQ         P7_SPIRQ(17)

/*MPEG_TS IRQs*/
#define P7_MPEGTS0_IRQ     P7_SPIRQ(18)
#define P7_MPEGTS1_IRQ     P7_SPIRQ(19)

/* CRYPTO */
#define P7_CRYPTO_IRQ       P7_SPIRQ(20)

/* DMAC IRQs on P7 R2/3 */
#define P7_DMA_IRQ          P7_SPIRQ(21)

/* AVI IRQs: 22 to 56 */
#define P7_CFG_IRQ          P7_SPIRQ(22)
#define P7_INTER_IRQ        P7_SPIRQ(23)
#define P7_FIFO00_IRQ       P7_SPIRQ(24)
#define P7_FIFO01_IRQ       P7_SPIRQ(25)
#define P7_FIFO02_IRQ       P7_SPIRQ(26)
#define P7_FIFO03_IRQ       P7_SPIRQ(27)
#define P7_FIFO04_IRQ       P7_SPIRQ(28)
#define P7_FIFO05_IRQ       P7_SPIRQ(29)
#define P7_FIFO06_IRQ       P7_SPIRQ(30)
#define P7_FIFO07_IRQ       P7_SPIRQ(31)
#define P7_FIFO08_IRQ       P7_SPIRQ(32)
#define P7_FIFO09_IRQ       P7_SPIRQ(33)
#define P7_FIFO10_IRQ       P7_SPIRQ(34)
#define P7_FIFO11_IRQ       P7_SPIRQ(35)
#define P7_CONV0_IRQ        P7_SPIRQ(36)
#define P7_CONV1_IRQ        P7_SPIRQ(37)
#define P7_CONV2_IRQ        P7_SPIRQ(38)
#define P7_CONV3_IRQ        P7_SPIRQ(39)
#define P7_BLEND0_IRQ       P7_SPIRQ(40)
#define P7_BLEND1_IRQ       P7_SPIRQ(41)
#define P7_LCD0_IRQ         P7_SPIRQ(42)
#define P7_LCD1_IRQ         P7_SPIRQ(43)
#define P7_CAM0_IRQ         P7_SPIRQ(44)
#define P7_CAM1_IRQ         P7_SPIRQ(45)
#define P7_CAM2_IRQ         P7_SPIRQ(46)
#define P7_CAM3_IRQ         P7_SPIRQ(47)
#define P7_CAM4_IRQ         P7_SPIRQ(48)
#define P7_CAM5_IRQ         P7_SPIRQ(49)
#define P7_SCALROT0_IRQ     P7_SPIRQ(50)
#define P7_SCALROT1_IRQ     P7_SPIRQ(51)
#define P7_GAM0_IRQ         P7_SPIRQ(52)
#define P7_GAM1_IRQ         P7_SPIRQ(53)

#define P7_VDEC_IRQ         P7_SPIRQ(57)
#define P7_VENC_IRQ         P7_SPIRQ(58)

#define P7_UART0_IRQ        P7_SPIRQ(59)
#define P7_UART1_IRQ        P7_SPIRQ(60)
#define P7_UART2_IRQ        P7_SPIRQ(61)
#define P7_UART3_IRQ        P7_SPIRQ(62)
#define P7_UART4_IRQ        P7_SPIRQ(63)
#define P7_UART5_IRQ        P7_SPIRQ(64)
#define P7_UART6_IRQ        P7_SPIRQ(65)
#define P7_UART7_IRQ        P7_SPIRQ(66)
#define P7_I2CM0_IRQ        P7_SPIRQ(67)
#define P7_I2CM1_IRQ        P7_SPIRQ(68)
#define P7_I2CM2_IRQ        P7_SPIRQ(69)
#define P7_I2CMS_IRQ        P7_SPIRQ(95)
#define P7_I2CS0_IRQ        P7_SPIRQ(70)
#define P7_I2CS1_IRQ        P7_SPIRQ(71)
#define P7_I2CS2_IRQ        P7_SPIRQ(72)
#define P7_GPIO_IRQ         P7_SPIRQ(73)

#define P7_AXIMON_IRQ	    P7_SPIRQ(74)

#define P7_TIM0_IRQ         P7_SPIRQ(75)
#define P7_TIM1_IRQ         P7_SPIRQ(76)
#define P7_TIM2_IRQ         P7_SPIRQ(77)
#define P7_TIM3_IRQ         P7_SPIRQ(78)

#define P7_AAI_IRQ          P7_SPIRQ(79)

#define P7_PWM_SERVO_RX_IRQ        P7_SPIRQ(82)

#define P7_CCAN0_IRQ        P7_SPIRQ(83)
#define P7_CCAN1_IRQ        P7_SPIRQ(84)


#define P7_PMU_IRQ0         P7_SPIRQ(115)
#define P7_PMU_IRQ1         P7_SPIRQ(116)

#define P7_IRQS             (160)

/* Parrot GPIO controller may handle up to 20 GPIO lines as interrupts. */
#if defined(CONFIG_GPIO_PARROT7) || defined(CONFIG_GPIO_PARROT7_MODULE)
#define P7_GPIO_IRQS        (20)
#else
#define P7_GPIO_IRQS        (0)
#endif

#define P7MU_GPIO_IRQS		(22)

#if defined(CONFIG_MACH_PARROT_FC7100)
/* IRQs for the two IO expanders */
#define P7_EXTERNAL_IRQS    (2 * 16)
#else
#define P7_EXTERNAL_IRQS    (0)
#endif

#define NR_IRQS               (P7_IRQS + P7_GPIO_IRQS + P7MU_GPIO_IRQS + P7_EXTERNAL_IRQS)

#endif
