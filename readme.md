LoRa Gateway
=====================

#### MAC layer

```c

typedef struct
{
    uint8_t mhdr;
    uint8_t control;
    uint16_t devaddr_lo;
    uint16_t devaddr_hi;
    uint16_t counter;
    uint8_t data[];
} mac_data_t;

```

#### Interface with LoRa network server


###### Packet received

```c
typedef struct
{
    uint8_t sf;
    uint8_t cr;
    uint16_t bw;
    uint32_t freq;
    int16_t snr;
    int16_t rssi;
    uint32_t second;
    uint32_t nanosecond;
    uint8_t len;
    uint8_t buf[256];
} rx_info_t;
```


###### Send packet (TODO)

```c

typedef struct
{
    uint8_t sf;
    uint8_t cr;
    uint16_t bw;
    uint32_t freq;
    uint8_t txpower;
    uint8_t len;
    uint8_t buf[256];
} tx_info_t;

```
