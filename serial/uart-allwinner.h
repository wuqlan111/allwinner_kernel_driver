#ifndef  GPIO_ALLWINNER_H6_H
#define  GPIO_ALLWINNER_H6_H

#include  <linux/bitops.h>

#define  UART_ALLWINNER_MAJOR	207
#define  UART_ALLWINNER_MINOR_START		50
#define  UART_ALLWINNER_NR_PORTS		4
#define  UART_ALLWINNER_FIFO_SIZE		(256u)
#define  UART_ALLWINNER_DRV_NAME        "allwinner-uart"
#define  UART_ALLWINNER_DEV_NAME        "ttyallwinner"
#define  UART_ALLWINNER_MAP_SIZE        (0x400u)

#define  UART_ALLWINNER_CONSOLE        NULL

#define ALLWINNER_UART_IER_PTIME        BIT(7)
#define ALLWINNER_UART_IER_RS485        BIT(4)
#define ALLWINNER_UART_IER_EDSSI        BIT(3)
#define ALLWINNER_UART_IER_ELSI         BIT(2)
#define ALLWINNER_UART_IER_ETBEI        BIT(1)
#define ALLWINNER_UART_IER_ERBFI        BIT(0)

#define ALLWINNER_UART_IIR_FEFLAG        GENMASK(7,  6)
#define ALLWINNER_UART_IIR_IID           GENMASK(3,  0)

#define ALLWINNER_UART_FCR_RT            GENMASK(7,  6)
#define ALLWINNER_UART_FCR_TFT           GENMASK(5,  4)
#define ALLWINNER_UART_FCR_DMAM          BIT(3)
#define ALLWINNER_UART_FCR_XFIFOR        BIT(2)
#define ALLWINNER_UART_FCR_RFIFOR        BIT(1)
#define ALLWINNER_UART_FCR_FIFOE         BIT(0)


#define ALLWINNER_UART_LCR_DLAB           BIT(7)
#define ALLWINNER_UART_LCR_BC             BIT(6)
#define ALLWINNER_UART_LCR_EPS            GENMASK(5,  4)
#define ALLWINNER_UART_LCR_PEN            BIT(3)
#define ALLWINNER_UART_LCR_STOP           BIT(2)
#define ALLWINNER_UART_LCR_DLEN           GENMASK(1,  0)


#define  ALLWINNER_UART_MCR_USART_FUNCTION            GENMASK(7, 6)
#define  ALLWINNER_UART_MCR_AFCE                      BIT(5)
#define  ALLWINNER_UART_MCR_LOOP                      BIT(4)
#define  ALLWINNER_UART_MCR_RTS                       BIT(1)
#define  ALLWINNER_UART_MCR_DTR                       BIT(0)


#define  ALLWINNER_UART_LSR_FIFOERR                      BIT(7)
#define  ALLWINNER_UART_LSR_TEMT                         BIT(6)
#define  ALLWINNER_UART_LSR_THRE                         BIT(5)
#define  ALLWINNER_UART_LSR_RI                           BIT(4)
#define  ALLWINNER_UART_LSR_FE                           BIT(3)
#define  ALLWINNER_UART_LSR_PE                           BIT(2)
#define  ALLWINNER_UART_LSR_OE                           BIT(1)
#define  ALLWINNER_UART_LSR_DR                           BIT(0)


#define  ALLWINNER_UART_MSR_DCD                          BIT(7)
#define  ALLWINNER_UART_MSR_RI                           BIT(6)
#define  ALLWINNER_UART_MSR_DSR                          BIT(5)
#define  ALLWINNER_UART_MSR_CTS                          BIT(4)
#define  ALLWINNER_UART_MSR_DDCD                         BIT(3)
#define  ALLWINNER_UART_MSR_TERI                         BIT(2)
#define  ALLWINNER_UART_MSR_DDSR                         BIT(1)
#define  ALLWINNER_UART_MSR_DCTS                         BIT(0)


#define  ALLWINNER_UART_SR_RFF                           BIT(4)
#define  ALLWINNER_UART_SR_RFNE                          BIT(3)
#define  ALLWINNER_UART_SR_TFE                           BIT(2)
#define  ALLWINNER_UART_SR_TFNF                          BIT(1)
#define  ALLWINNER_UART_SR_BUSY                          BIT(0)






#endif
