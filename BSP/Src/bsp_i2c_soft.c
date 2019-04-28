
#include "stm32f1xx_hal.h"

#include "bsp_i2c_soft.h"

#define ACK   (0)
#define NACK  (1)

#define WRITE (0)
#define READ  (1)

#define SCL_GPIO(p) ((GPIO_TypeDef *)p->SCL.GPIO)
#define SCL_Pin(p)  (p->SCL.Pin)
#define SCL_High(p) (SCL_GPIO(p)->BSRR = SCL_Pin(p))
#define SCL_Low(p)  (SCL_GPIO(p)->BSRR = SCL_Pin(p) << 16)

#define SDA_GPIO(p) ((GPIO_TypeDef *)p->SDA.GPIO)
#define SDA_Pin(p)  (p->SDA.Pin)
#define SDA_High(p) (SDA_GPIO(p)->BSRR = SDA_Pin(p))
#define SDA_Low(p)  (SDA_GPIO(p)->BSRR = SDA_Pin(p) << 16)
#define SDA_Read(p) (SDA_GPIO(p)->IDR & SDA_Pin(p))

static int Delay    (volatile int);
static int Start    (void *);
static int Stop     (void *);
static int SendByte (void *, int);
static int RecvByte (void *);
static int SendACK  (void *, int);
static int RecvACK  (void *);

static int Delay(volatile int delay)
{
    while (delay--);

    return 0;
}

static int Start(void *pPort)
{
    I2C_PortType *ptrPort = pPort;

    SDA_High(ptrPort);
    Delay(ptrPort->Delay);
    SCL_High(ptrPort);
    Delay(ptrPort->Delay);
    SDA_Low(ptrPort);
    Delay(ptrPort->Delay);
    SCL_Low(ptrPort);
    Delay(ptrPort->Delay);

    return 0;
}

static int Stop(void *pPort)
{
    I2C_PortType *ptrPort = pPort;

    SDA_Low(ptrPort);
    Delay(ptrPort->Delay);
    SCL_High(ptrPort);
    Delay(ptrPort->Delay);
    SDA_High(ptrPort);
    Delay(ptrPort->Delay);

    return 0;
}

static int SendByte(void *pPort, int byte)
{
    I2C_PortType *ptrPort = pPort;

    for(int i = 0; i < 8; i++)
    {
        if (byte & 0x80)
            SDA_High(ptrPort);
        else
            SDA_Low(ptrPort);
        Delay(ptrPort->Delay);
        SCL_High(ptrPort);
        Delay(ptrPort->Delay * 2);
        SCL_Low(ptrPort);
        Delay(ptrPort->Delay);
        byte <<= 1;
    }

    return 0;
}

static int RecvByte(void *pPort)
{
    I2C_PortType *ptrPort = pPort;
    int byte = 0;

    for(int i = 0; i < 8; i++)
    {
        Delay(ptrPort->Delay);
        SCL_High(ptrPort);
        Delay(ptrPort->Delay);
        byte <<= 1;
        if (SDA_Read(ptrPort))
            byte++;
        Delay(ptrPort->Delay);
        SCL_Low(ptrPort);
        Delay(ptrPort->Delay);
    }

    return byte;
}

static int SendACK(void *pPort, int ack)
{
    I2C_PortType *ptrPort = pPort;

    if (ack == 0)
        SDA_Low(ptrPort);
    else
        SDA_High(ptrPort);
    Delay(ptrPort->Delay);
    SCL_High(ptrPort);
    Delay(ptrPort->Delay * 2);
    SCL_Low(ptrPort);
    Delay(ptrPort->Delay);
    if (ack == 0)
        SDA_High(ptrPort);

    return 0;
}

static int RecvACK(void *pPort)
{
    I2C_PortType *ptrPort = pPort;
    int ack = 0;

    Delay(ptrPort->Delay);
    SCL_High(ptrPort);
    Delay(ptrPort->Delay);
    if (SDA_Read(ptrPort))
        ack++;
    Delay(ptrPort->Delay);
    SCL_Low(ptrPort);
    Delay(ptrPort->Delay);

    return ack;
}

static int I2C_Init(void *pPort)
{
    I2C_PortType *ptrPort = pPort;
    GPIO_InitTypeDef GPIO_InitStruct;

    if (ptrPort == NULL)
        return -1;

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;

    GPIO_InitStruct.Pin   = SCL_Pin(ptrPort);
    HAL_GPIO_Init(SCL_GPIO(ptrPort), &GPIO_InitStruct);

    SCL_High(ptrPort);

    GPIO_InitStruct.Pin   = SDA_Pin(ptrPort);
    HAL_GPIO_Init(SDA_GPIO(ptrPort), &GPIO_InitStruct);

    SDA_High(ptrPort);

    return 0;
}

static int I2C_DataRdWr(void *pPort, int RdWr, int DevAddr, void *pData, int length)
{
    I2C_PortType  *ptrPort = pPort;
    unsigned char *ptrData = pData;
    int            result  = -1;

    if ((ptrPort == NULL) || (ptrData == NULL))
        goto RETURN;

    if (ptrPort->Busy != 0)
    {
        result = -2;
        goto RETURN;
    }

    ptrPort->Busy = 1;

    Start(ptrPort);

    if (RdWr == 0)
        SendByte(ptrPort, DevAddr | WRITE);
    else
        SendByte(ptrPort, DevAddr | READ);

    if (RecvACK(ptrPort))
        goto STOP;

    if (RdWr == 0)
    {
        while (length--)
        {
            SendByte(ptrPort, *ptrData++);
            if (RecvACK(ptrPort))
                goto STOP;
        }
    }
    else
    {
        while (length--)
        {
            *ptrData++ = RecvByte(ptrPort);
            if (length)
                SendACK(ptrPort, ACK);
            else
                SendACK(ptrPort, NACK);
        }
    }

    result = 0;

  STOP:
    Stop(ptrPort);

    ptrPort->Busy = 0;

  RETURN:
    return result;
}

static int I2C_MemoryRdWr(void *pPort, int RdWr, int DevAddr, int MemLens, int MemAddr, void *pData, int length)
{
    I2C_PortType  *ptrPort = pPort;
    unsigned char *ptrData = pData;
    int            result  = -1;

    if ((MemLens < 1) || (MemLens > 2))
        goto RETURN;

    if ((ptrPort == NULL) || (ptrData == NULL))
        goto RETURN;

    if (ptrPort->Busy != 0)
    {
        result = -2;
        goto RETURN;
    }

    ptrPort->Busy = 1;

    Start(ptrPort);

    SendByte(ptrPort, DevAddr | WRITE);

    if (RecvACK(ptrPort))
        goto STOP;

    while (MemLens--)
    {
        SendByte(ptrPort, 0xFF & (MemAddr >> (8 * MemLens)));
        if (RecvACK(ptrPort))
            goto STOP;
    }

    if (RdWr == 0)
    {
        while (length--)
        {
            SendByte(ptrPort, *ptrData++);
            if (RecvACK(ptrPort))
                goto STOP;
        }
    }
    else
    {
        Start(ptrPort);

        SendByte(ptrPort, DevAddr | READ);

        if (RecvACK(ptrPort))
            goto STOP;

        while (length--)
        {
            *ptrData++ = RecvByte(ptrPort);
            if (length)
                SendACK(ptrPort, ACK);
            else
                SendACK(ptrPort, NACK);
        }
    }

    result = 0;

  STOP:
    Stop(ptrPort);

    ptrPort->Busy = 0;

  RETURN:
    return result;
}

void *BSP_I2C_OBJ(void)
{
	static I2C_SOFT_OBJ obj = {
		.Init       = I2C_Init,
		.DataRdWr   = I2C_DataRdWr,
		.MemoryRdWr = I2C_MemoryRdWr,
	};

	return &obj;
}
