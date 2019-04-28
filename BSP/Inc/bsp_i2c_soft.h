
#ifndef _BSP_I2C_SOFT_H_
#define _BSP_I2C_SOFT_H_

struct _I2C_Port {
    void *GPIO;
    int   Pin;
};

typedef struct _I2C_PortType {
    struct _I2C_Port SDA;
    struct _I2C_Port SCL;
    int              Delay;
    int              Busy;
} I2C_PortType;

typedef struct _I2C_SOFT_OBJ {
	int (*Init)(void *);
	int (*DataRdWr)(void *, int, int, void *, int);
	int (*MemoryRdWr)(void *, int, int, int, int, void *, int);
} I2C_SOFT_OBJ;

void *BSP_I2C_OBJ(void);

#endif

