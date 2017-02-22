#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "radio.h"

char static *regname[] = {"RegFifo", "RegOpMode", "N/A", "N/A", "N/A", "N/A", "RegFrfMsb", \
"RegFrfMid", "RegFrfLsb", "RegPaConfig", "RegPaRamp", "RegOcp", "RegLna", \
"LORARegFifoAddrPtr", "LORARegFifoTxBaseAddr", "LORARegFifoRxBaseAddr", \
"LORARegFifoRxCurrentAddr", "LORARegIrqFlagsMask", "LORARegIrqFlags", \
"LORARegRxNbBytes", "LORARegRxHeaderCntValueMsb", "LORARegRxHeaderCntValueLsb", \
"LORARegRxPacketCntValueMsb", "LORARegRxpacketCntValueLsb", "LORARegModemStat", \
"LORARegPktSnrValue", "LORARegPktRssiValue", "LORARegRssiValue", "LORARegHopChannel", \
"LORARegModemConfig1", "LORARegModemConfig2", "LORARegSymbTimeoutLsb", \
"LORARegPreambleMsb", "LORARegPreambleLsb", "LORARegPayloadLength", \
"LORARegPayloadMaxLength", "LORARegHopPeriod", "LORARegFifoRxByteAddr", \
"LORARegModemConfig3", "N/A", "LORARegFeiMsb", "LORAFeiMib", "LORARegFeiLsb", \
"N/A", "LORARegRssiWideband", "N/A", "N/A", "N/A", "N/A", "LORARegDetectOptimize", \
"N/A", "LORARegInvertIQ", "N/A", "N/A", "N/A", "LORARegDetectionThreshold", "N/A", \
"LORARegSyncWord", "FSKRegTimer2Coef", "FSKRegImageCal", "N/A", "N/A", "N/A", "N/A", "RegDioMapping1", \
"RegDioMapping2", "RegVersion"};


void static pin_init()
{
    wiringPiSetup();
    pinMode(PIN_NSS, OUTPUT);
    pinMode(PIN_DIO0, INPUT);
    pinMode(PIN_DIO1, INPUT);
    pinMode(PIN_DIO2, INPUT);
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_NSS, HIGH);
    digitalWrite(PIN_RST, HIGH);
    wiringPiSPISetup(SPI_CHANNEL, SPI_FREQ);
}

void static pin_cleanup()
{
    pinMode(PIN_NSS, INPUT);
    pinMode(PIN_DIO0, INPUT);
    pinMode(PIN_DIO1, INPUT);
    pinMode(PIN_DIO2, INPUT);
    pinMode(PIN_RST, INPUT);
}

void static select_chip()
{
    digitalWrite(PIN_NSS, LOW);
}

void static unselect_chip()
{
    digitalWrite(PIN_NSS, HIGH);
}

uint8_t static read_byte(uint8_t addr)
{
    unsigned char spibuf[2];

    select_chip();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(SPI_CHANNEL, spibuf, 2);
    unselect_chip();

    fprintf(stderr, "Read 0x%x from %s\n", spibuf[1], regname[addr]);

    return spibuf[1];
}

void static write_byte(uint8_t addr, uint8_t value)
{
    unsigned char spibuf[2];
    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    select_chip();
    wiringPiSPIDataRW(SPI_CHANNEL, spibuf, 2);
    unselect_chip();
    fprintf(stderr, "Wrote 0x%x to %s\n", value, regname[addr]);
}

static void write_fifo (uint8_t* buf, uint8_t len)
{
    uint8_t addr = RegFifo | 0x80;
    select_chip();
    wiringPiSPIDataRW(SPI_CHANNEL, &addr, 1);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, len);
    unselect_chip();
    fprintf(stderr, "Wrote %u bytes to 0x%x\n", len, addr & 0x7f);
}

static void read_fifo (uint8_t* buf, uint8_t len)
{
    uint8_t addr = RegFifo & 0x7f;
    select_chip();
    memset(buf, 0, sizeof(uint8_t) * len);
    wiringPiSPIDataRW(SPI_CHANNEL, &addr, 1);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, len);
    unselect_chip();
    fprintf(stderr, "Read %u bytes from 0x%x\n", len, addr & 0x7f);
}

static void lora_reset()
{
    digitalWrite(PIN_RST, HIGH);
    delay(100);
    digitalWrite(PIN_RST, LOW);
    delay(100);
}

static uint8_t lora_get_version()
{
    return read_byte(RegVersion);
}

static void lora_set_opmode(uint8_t opmode)
{
    write_byte(RegOpMode, opmode | OPMODE_LORA | OPMODE_LOWFREQON);
}

static void dump_dio()
{
    fprintf(stderr, "DIO0 = %d, ", digitalRead(PIN_DIO0));
    fprintf(stderr, "DIO1 = %d, ", digitalRead(PIN_DIO1));
    fprintf(stderr, "DIO2 = %d\n", digitalRead(PIN_DIO2));
}

int lora_get_last_packet_coding_rate()
{
    int cr = (read_byte(LORARegModemStat) >> 5) & 0x07;
    switch (cr)
    {
        case 1: return 45;
        case 2: return 46;
        case 3: return 47;
        case 4: return 48;
        default: fprintf(stderr, "Unknown CR = %d\n", cr);
    }
    return 0;
}

int lora_get_last_packet_snr()
{
    return (int8_t)read_byte(LORARegPktSnrValue) >> 2;
}

int lora_get_last_packet_rssi()
{
    return -164 + read_byte(LORARegPktRssiValue);
}

int lora_get_current_rssi()
{
    return -164 + read_byte(LORARegRssiValue);
}

int lora_get_modem_status()
{
    return (read_byte(LORARegModemStat) & 0x1f);
}

void lora_set_frequency(long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    write_byte(RegFrfMsb, (uint8_t)(frf>>16));
    write_byte(RegFrfMid, (uint8_t)(frf>>8));
    write_byte(RegFrfLsb, (uint8_t)(frf));
}

long lora_get_frequency()
{
    long long frf = 0;
    frf = read_byte(RegFrfLsb);
    frf |= ((long)read_byte(RegFrfMid) << 8);
    frf |= ((long)read_byte(RegFrfMsb) << 16);

    frf = (frf * 32000000) >> 19;
    return (long)frf;
}

void static lora_set_rx_timeout(int symbols)
{
    assert(symbols >= 0);
    assert(symbols <= 0x3ff);

    uint8_t val;
    val = (read_byte(LORARegModemConfig2) & 0xfc) | ((symbols >> 8) & 0x3);
    write_byte(LORARegModemConfig2, val);
    write_byte(LORARegSymbTimeoutLsb, (uint8_t)(val & 0xff));
}

void lora_tx(uint8_t *data, uint8_t len)
{
    uint8_t flags;
    int state;

    fprintf(stderr, "Sending data of %d bytes.\n", len);
    lora_set_opmode(OPMODE_STANDBY);

    write_byte(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    write_byte(LORARegIrqFlags, 0xFF);
    write_byte(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    write_byte(LORARegFifoTxBaseAddr, 0);
    write_byte(LORARegFifoAddrPtr, 0);
    write_byte(LORARegPayloadLength, len);

    write_fifo(data, len);
    dump_dio();

    lora_set_opmode(OPMODE_TX); //start sending here

    do {
        state = digitalRead(PIN_DIO0);
    } while(state == 0);

    dump_dio();

    flags = read_byte(LORARegIrqFlags);
    assert(flags & IRQ_LORA_TXDONE_MASK);
    write_byte(LORARegIrqFlagsMask, 0xFF);
    write_byte(LORARegIrqFlags, 0xFF);
    fprintf(stderr, "Data sent\n");
}

// return the length of received packet
// 0 for timeout
int lora_rx_single(uint8_t *buf, int timeout_symbols)
{
    int len;
    int state;
    uint8_t flags;

    lora_set_opmode(OPMODE_STANDBY);

    if (timeout_symbols > 0)
    {
        lora_set_rx_timeout(timeout_symbols);
    }

    write_byte(RegLna, LNA_RX_GAIN);
    write_byte(LORARegPayloadMaxLength, 64);

    write_byte(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP);
    // clear flags
    write_byte(LORARegIrqFlags, 0xFF);
    write_byte(LORARegIrqFlagsMask, (uint8_t)(~(IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK)));
    dump_dio();
    // start receiving
    lora_set_opmode(OPMODE_RX_SINGLE);

    // wait for rxdone or timeout
    do {
        state = digitalRead(PIN_DIO0) | digitalRead(PIN_DIO1);
    }
    while(state == 0);
    dump_dio();
    // check flags
    flags = read_byte(LORARegIrqFlags);
    if (flags & IRQ_LORA_RXDONE_MASK)
    {
        len = read_byte(LORARegRxNbBytes);
        // put fifo pointer to last packet
        write_byte(LORARegFifoAddrPtr, read_byte(LORARegFifoRxCurrentAddr));
        // copy to buffer
        read_fifo(buf, len);
    }
    else if (flags & IRQ_LORA_RXTOUT_MASK)
    {
        len = 0;
    }
    else
    {
        len = -1;
        fprintf(stderr, "No RX flags\n");
    }
    write_byte(LORARegIrqFlagsMask, 0xFF);
    write_byte(LORARegIrqFlags, 0xFF);

    return len;
}

void lora_init()
{
    fprintf(stderr, "Init...\n");
    pin_init();
    lora_reset();
    assert(lora_get_version() == 0x12);
    lora_set_opmode(OPMODE_SLEEP);

    // //calibrate
    // write_byte(RegPaConfig, 0);
    // write_byte(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
    // while((read_byte(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    write_byte(LORARegIrqFlags, 0xFF);
    write_byte(LORARegIrqFlagsMask, 0xFF);
    fprintf(stderr, "Inited.\n");
}


// always explicit mode, SF >= 7
int lora_config(int sf, int cr, int bw, int prelen, uint8_t syncword)
{
    uint8_t mc1 = 0;
    uint8_t mc2 = 0;

    switch (bw)
    {
        case 125: mc1 |= 0x70; break;
        case 250: mc1 |= 0x80; break;
        case 500: mc1 |= 0x90; break;
        default: fprintf(stderr, "Unknown BW = %d\n", bw); return -1;
    }

    switch (cr) {
        case 45: mc1 |= 0x02; break;
        case 46: mc1 |= 0x04; break;
        case 47: mc1 |= 0x06; break;
        case 48: mc1 |= 0x08; break;
        default: fprintf(stderr, "Unknown CR = %d\n", cr); return -1;
    }

    if (sf >= 7 && sf <= 12)
    {
        mc2 |= (sf << 4) & 0xf0;
    }
    else
    {
        fprintf(stderr, "Unknown SF = %d\n", sf); return -1;
    }
    // set mc1 and mc2
    write_byte(LORARegModemConfig1, mc1);
    write_byte(LORARegModemConfig2, mc2);
    // set preamble length
    write_byte(LORARegPreambleMsb, (prelen >> 8) & 0xff);
    write_byte(LORARegPreambleLsb, (prelen) & 0xff);
    // set sync word
    write_byte(LORARegSyncWord, syncword);
    return 0;
}

void lora_cleanup()
{
    fprintf(stderr, "Cleaning up.\n");
    lora_set_opmode(OPMODE_SLEEP);
    pin_cleanup();
}


void dump_hex(void *data, int len)
{
    uint8_t *ptr = data;
    int i;
    for (i = 0; i < len; i++)
    {
        printf("%2x", ptr[i]);
    }
    printf("\n");
}
