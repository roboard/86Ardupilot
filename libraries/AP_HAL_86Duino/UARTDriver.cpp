
#include "UARTDriver.h"

namespace x86Duino {

#if defined (__86DUINO_AI)

#define  COM1_TX    (0x9C)
#define  COM1_RX    (0x9D)
#define  COM2_TX    (0x9E)
#define  COM2_RX    (0x9F)
#define  COM3_TX    (0x98)
#define  COM3_RX    (0x99)

#else

#define  COM1_TX    (0x9A)
#define  COM1_RX    (0x9B)
#define  COM2_TX    (0x9E)
#define  COM2_RX    (0x9F)
#define  COM3_TX    (0x9C)
#define  COM3_RX    (0x9D)

#endif

UARTDriver::UARTDriver(int com_port, unsigned long com_baudrate, unsigned char com_format, unsigned long com_rxtimeout, unsigned long com_txtimeout)
{
    port        = com_port;
    baudrate    = com_baudrate;
    format      = com_format;
    rxtimeout   = com_rxtimeout;
    txtimeout   = com_txtimeout;
    peek_stored = false;
    handle      = NULL;
}

void UARTDriver::begin(uint32_t baud)
{
    begin(baud, 100, 100);
}

void UARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS)
{
    unsigned short crossbar_ioaddr = 0;
    // close uart if opened
    if(handle != NULL)
    {
        end();  // close it and re-open
    }

    if (io_Init() == false) {
        printf("ERROR: IO init fail.\n");
        return;
    }
    sb_Write(0xc0, (sb_Read(0xc0) & 0x7fffffffL) | ((unsigned long)1L << 31));
    io_Close();

    if ((handle = com_Init(port)) == NULL)
    {
        printf("COM init fail!!\n");
        return;
    }

    switch (baud) {
    case 6000000L: baud = COM_UARTBAUD_6000000BPS; break;
    case 3000000L: baud = COM_UARTBAUD_3000000BPS; break;
    case 2000000L: baud = COM_UARTBAUD_2000000BPS; break;
    case 1500000L: baud = COM_UARTBAUD_1500000BPS; break;
    case 1000000L: baud = COM_UARTBAUD_1000000BPS; break;
    case 750000L:  baud = COM_UARTBAUD_750000BPS;  break;
    case 500000L:  baud = COM_UARTBAUD_500000BPS;  break;
    case 461538L:  baud = COM_UARTBAUD_461538BPS;  break;
    case 333333L:  baud = COM_UARTBAUD_333333BPS;  break;
    case 300000L:  baud = COM_UARTBAUD_300000BPS;  break;
    case 250000L:  baud = COM_UARTBAUD_250000BPS;  break;
    case 200000L:  baud = COM_UARTBAUD_200000BPS;  break;
    case 150000L:  baud = COM_UARTBAUD_150000BPS;  break;
    case 125000L:  baud = COM_UARTBAUD_125000BPS;  break;
    case 115200L:  baud = COM_UARTBAUD_115200BPS;  break;
    case 57600L:   baud = COM_UARTBAUD_57600BPS;   break;
    case 38400L:   baud = COM_UARTBAUD_38400BPS;   break;
    case 28800L:   baud = COM_UARTBAUD_28800BPS;   break;
    case 19200L:   baud = COM_UARTBAUD_19200BPS;   break;
    case 14400L:   baud = COM_UARTBAUD_14400BPS;   break;
    case 9600L:    baud = COM_UARTBAUD_9600BPS;    break;
    case 4800L:    baud = COM_UARTBAUD_4800BPS;    break;
    case 2400L:    baud = COM_UARTBAUD_2400BPS;    break;
    case 1200L:    baud = COM_UARTBAUD_1200BPS;    break;
    case 600L:     baud = COM_UARTBAUD_600BPS;     break;
    case 300L:     baud = COM_UARTBAUD_300BPS;     break;
    case 50L:      baud = COM_UARTBAUD_50BPS;      break;
    default:       baud = COM_UARTBAUD_9600BPS;    break;
    }

    com_SetBPS(handle, baud);
    com_SetFormat(handle, format);
    com_FlushTxQueue(handle);
    com_FlushRxQueue(handle);
    com_SetTimeOut(handle, rxtimeout, txtimeout);
    crossbar_ioaddr = sb_Read16(0x64)&0xfffe;

#if defined (__86DUINO_ZERO) || defined (__86DUINO_ONE) || defined (__86DUINO_EDUCAKE) || defined (__86DUINO_AI)
//    if(port == COM1 || port == COM2 || port == COM3)
//        if(comtype == COM_HalfDuplex) com_EnableHalfDuplex(handle);

    if(port == COM1)
    {
        io_outpb(crossbar_ioaddr + COM1_TX, 0x08);
        io_outpb(crossbar_ioaddr + COM1_RX, 0x08);
    }
    else if(port == COM2)
    {
        io_outpb(crossbar_ioaddr + COM2_TX, 0x08);
        io_outpb(crossbar_ioaddr + COM2_RX, 0x08);
    }
    else if(port == COM3)
    {
        io_outpb(crossbar_ioaddr + COM3_TX, 0x08);
        io_outpb(crossbar_ioaddr + COM3_RX, 0x08);
    }
#endif
}

void UARTDriver::end()
{
    if(handle == NULL) return;
    com_FlushWFIFO(handle);
    com_Close(handle);
    handle = NULL;
}
void UARTDriver::flush()
{
    if(handle == NULL) return;
    com_FlushTxQueue(handle);
}

bool UARTDriver::is_initialized()
{
    return (handle == NULL ? 0 : 1 );
}

void UARTDriver::set_blocking_writes(bool blocking)
{
    // default is non-blocking for 86duino API
    // default Buffer size is 4k for Tx/Rx
}

bool UARTDriver::tx_pending()
{
    if(handle == NULL) return 0;
    return (!com_TxReady(handle));
}

void UARTDriver::set_flow_control(enum flow_control flow_control_setting)
{
    // No hardware flow control on Vx86EX
}

/* Empty implementations of Stream virtual methods */
uint32_t UARTDriver::available()
{
    if(handle == NULL) return 0;
    return com_QueryRxQueue(handle);
}

uint32_t UARTDriver::txspace()
{
    if(handle ==NULL) return 0;
    // caution! size define in uart.cpp
    return (4096 - com_QueryTxQueue(handle));
}

int UARTDriver::peek(void)
{
    if(handle == NULL) return -1;
    if(peek_stored == true)
        return peek_val;
    else
    {
        if((peek_val = com_Read(handle)) == 0xFFFF)
            return -1;//peek_val = -1;
        peek_stored = true;
        return peek_val;
    }
}

int16_t UARTDriver::read()
{
    int c;
    if(handle == NULL) return -1;
    if(peek_stored == true)
    {
        peek_stored = false;
        return peek_val;
    }
    else
    {
        c = com_Read(handle);
        return (c == 0xFFFF) ? -1 : c;
    }
}

/* Empty implementations of Print virtual methods */
size_t UARTDriver::write(uint8_t c)
{
    if(handle == NULL) return 0;
    return (com_Write(handle, c) == true) ? 1 : 0;
}

size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

}
