#ifndef _P7_I2CS_REGS_H_
#define _P7_I2CS_REGS_H_

#define I2CS_ADDRESS                    0x00
#define I2CS_ITEN                       0x04
#define I2CS_ITACK                      0x08
#define I2CS_TRANSMIT                   0x0C
#define I2CS_FIFO_FLUSH                 0x10
#define I2CS_STATUS                     0x14
#define I2CS_TIMING                     0x18
#define I2CS_CONTROL                    0x1C
#define I2CS_RECEIVE                    0x20 /* 8 contiguous registers starting
                                              * at this address */

#define I2CS_ITEN_TX                    BIT(0)
#define I2CS_ITEN_RX                    BIT(1)
#define I2CS_ITEN_TRANSFER_DONE         BIT(2)

#define I2CS_ITACK_TX                   BIT(0)
#define I2CS_ITACK_RX                   BIT(1)
#define I2CS_ITACK_TRANSFER_DONE        BIT(2)
#define I2CS_ITACK_CORE_SLEEP           BIT(26)

#define I2CS_FIFO_FLUSH_TX              BIT(0)
#define I2CS_FIFO_FLUSH_RX              BIT(1)

#define I2CS_STATUS_IT_TX               BIT(0)
#define I2CS_STATUS_IT_RX               BIT(1)
#define I2CS_STATUS_IT_TRANSFER_DONE    BIT(2)
#define I2CS_STATUS_TX_EMPTY            BIT(8)
#define I2CS_STATUS_TX_FULL             BIT(9)
#define I2CS_STATUS_RX_EMPTY            BIT(16)
#define I2CS_STATUS_RX_FULL             BIT(17)
#define I2CS_STATUS_BUS_BUSY            BIT(24)
#define I2CS_STATUS_CORE_BUSY           BIT(25)
#define I2CS_STATUS_CORE_SLEEP          BIT(26)

#define I2CS_CONTROL_W_ACK_POLLING_MODE BIT(16)

#define I2CS_RECEIVE_INVAL              BIT(8)
#define I2CS_RECEIVE_STOP               BIT(12)

#endif /* _P7_I2CS_REGS_H_ */
