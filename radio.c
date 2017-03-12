#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "radio.h"
#include "sx1278.h"
#include "config.h"

#define SPI_START_TRANSCATION() while(0)
#define SPI_END_TRANSCATION()   while(0)

#define select_chip()   digitalWrite(PIN_NSS, LOW);
#define unselect_chip()    digitalWrite(PIN_NSS, HIGH);

static char *regname[] = {"RegFifo", "RegOpMode", "N/A", "N/A", "N/A", "N/A", "RegFrfMsb", \
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
"LORARegSyncWord", "FSKRegTimer2Coef", "FSKRegImageCal", "N/A", "N/A", "N/A", "N/A", \
"RegDioMapping1", "RegDioMapping2", "RegVersion"};


void static pin_init()
{
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


uint8_t static read_byte(uint8_t addr)
{
    unsigned char spibuf[2];
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;

    select_chip();
    wiringPiSPIDataRW(SPI_CHANNEL, spibuf, 2);
    unselect_chip();

    // fprintf(stderr, "Read 0x%x from %s\n", spibuf[1], regname[addr]);
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
    // fprintf(stderr, "Wrote 0x%x to %s\n", value, regname[addr]);
}

static void write_fifo (uint8_t* buf, uint8_t len)
{
    uint8_t addr = RegFifo | 0x80;

    select_chip();
    wiringPiSPIDataRW(SPI_CHANNEL, &addr, 1);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, len);
    unselect_chip();
    // fprintf(stderr, "Wrote %u bytes to 0x%x\n", len, addr & 0x7f);
}

static void read_fifo (uint8_t* buf, uint8_t len)
{
    uint8_t addr = RegFifo & 0x7f;
    memset(buf, 0, sizeof(uint8_t) * len);

    select_chip();
    wiringPiSPIDataRW(SPI_CHANNEL, &addr, 1);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, len);
    unselect_chip();

    // fprintf(stderr, "Read %u bytes from 0x%x\n", len, addr & 0x7f);
}

static void lora_reset()
{
    digitalWrite(PIN_RST, LOW);
    delay(100);
    digitalWrite(PIN_RST, HIGH);
    delay(100);
}

static uint8_t lora_get_version()
{
    uint8_t ver;
    SPI_START_TRANSCATION();
    ver = read_byte(RegVersion);
    SPI_END_TRANSCATION();
    return ver;
}

static void lora_set_opmode(uint8_t opmode)
{
    SPI_START_TRANSCATION();
    write_byte(RegOpMode, opmode | OPMODE_LORA | OPMODE_LOWFREQON);
    SPI_END_TRANSCATION();
}

static uint8_t lora_get_opmode()
{
    uint8_t mode;
    SPI_START_TRANSCATION();
    mode = read_byte(RegOpMode) & OPMODE_MASK;
    SPI_END_TRANSCATION();
    return mode;
}

static void dump_dio()
{
    fprintf(stderr, "DIO0 = %d, ", digitalRead(PIN_DIO0));
    fprintf(stderr, "DIO1 = %d, ", digitalRead(PIN_DIO1));
    fprintf(stderr, "DIO2 = %d\n", digitalRead(PIN_DIO2));
}

static void lora_set_rx_timeout(int symbols)
{
    assert(symbols >= 0);
    assert(symbols <= 0x3ff);

    uint8_t val;

    SPI_START_TRANSCATION();
    val = (read_byte(LORARegModemConfig2) & 0xfc) | ((symbols >> 8) & 0x3);
    write_byte(LORARegModemConfig2, val);
    write_byte(LORARegSymbTimeoutLsb, (uint8_t)(val & 0xff));
    SPI_END_TRANSCATION();
}

static void lora_set_invert_iq()
{
    fprintf(stderr, "Set invert.\n");

    SPI_START_TRANSCATION();
    write_byte(LORARegInvertIQ, read_byte(LORARegInvertIQ) | 0x40);
    assert(read_byte(LORARegInvertIQ) & 0x40);
    SPI_END_TRANSCATION();
}

static void lora_clear_invert_iq()
{
    fprintf(stderr, "Clear invert.\n");

    SPI_START_TRANSCATION();
    write_byte(LORARegInvertIQ, read_byte(LORARegInvertIQ) & 0xbf);
    assert((read_byte(LORARegInvertIQ) & 0x40) == 0);
    SPI_END_TRANSCATION();
}

static int lora_get_last_packet_coding_rate()
{
    int cr;
    SPI_START_TRANSCATION();
    cr = (read_byte(LORARegModemStat) >> 5) & 0x07;
    SPI_END_TRANSCATION();
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

static int lora_get_last_packet_snr()
{
    int ret;
    SPI_START_TRANSCATION();
    ret = (int8_t)read_byte(LORARegPktSnrValue) >> 2;
    SPI_END_TRANSCATION();
    return ret;
}

static int lora_get_last_packet_rssi()
{
    int ret;
    SPI_START_TRANSCATION();
    ret = read_byte(LORARegPktRssiValue);
    SPI_END_TRANSCATION();
    return -164 + ret;
}

static int lora_get_modem_status()
{
    int stat;
    SPI_START_TRANSCATION();
    stat = (read_byte(LORARegModemStat) & 0x1f);
    SPI_END_TRANSCATION();
    return stat;
}

static int lora_get_last_packet_crc()
{
    int crc;
    SPI_START_TRANSCATION();
    crc = read_byte(LORARegHopChannel) & 0x20;
    SPI_END_TRANSCATION();
    return crc;
}

// fill rx_info_t with current configuration and time
static int fill_rx_info_t(rx_info_t *data)
{
    uint8_t mc1, mc2;
    long long frf = 0;
    struct timespec tv;

    frf = read_byte(RegFrfLsb);
    frf |= ((long)read_byte(RegFrfMid) << 8);
    frf |= ((long)read_byte(RegFrfMsb) << 16);
    data->freq = (frf * 32000000) >> 19;    // full freq

    mc1 = read_byte(LORARegModemConfig1);
    mc2 = read_byte(LORARegModemConfig2);
    // mc3 = read_byte(LORARegModemConfig3);

    // fill bw
    switch(mc1 & 0xf0)
    {
        case 0x70: data->bw = 125; break;
        case 0x80: data->bw = 250; break;
        case 0x90: data->bw = 500; break;
        default: return -1;
    }

    // fill sf
    data->sf = (mc2 & 0xf0) >> 4;
    clock_gettime(CLOCK_REALTIME, &tv);

    // fill second and nanosecond
    data->second = tv.tv_sec;
    data->nanosecond = tv.tv_nsec;
    return 0;
}

/* Below are exported functions */

int lora_get_current_rssi()
{
    int rssi;
    SPI_START_TRANSCATION();
    rssi = -164 + read_byte(LORARegRssiValue);
    SPI_END_TRANSCATION();
    return rssi;
}

int lora_set_frequency(long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    if (freq < 435000000 && freq > 430000000)
    {
        piLock(COMMAND_LOCK_NUMBER);
        SPI_START_TRANSCATION();
        write_byte(RegFrfMsb, (uint8_t)(frf>>16));
        write_byte(RegFrfMid, (uint8_t)(frf>>8));
        write_byte(RegFrfLsb, (uint8_t)(frf));
        SPI_END_TRANSCATION();
        piUnlock(COMMAND_LOCK_NUMBER);
        return 0;
    }
    return -1;
}

long lora_get_frequency()
{
    long long frf = 0;

    piLock(COMMAND_LOCK_NUMBER);
    frf = read_byte(RegFrfLsb);
    frf |= ((long)read_byte(RegFrfMid) << 8);
    frf |= ((long)read_byte(RegFrfMsb) << 16);
    piUnlock(COMMAND_LOCK_NUMBER);

    frf = (frf * 32000000) >> 19;
    return (long)frf;
}

/* send data of len */
/* !!! data will be modified after SPI transaction !! */
int lora_tx(uint8_t *data, uint8_t len)
{
    uint8_t flags;
    int state;

    // fprintf(stderr, "Sending data of %d bytes.\n", len);
    piLock(COMMAND_LOCK_NUMBER);
    lora_set_opmode(OPMODE_STANDBY);

#ifdef CONFIG_LORA_IS_GATEWAY
    // set invert I/Q bit
    lora_set_invert_iq();
#else
    // clear invert I/Q bit
    lora_clear_invert_iq();
#endif

    write_byte(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    write_byte(LORARegIrqFlags, 0xFF);
    write_byte(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    write_byte(LORARegFifoTxBaseAddr, 0);
    write_byte(LORARegFifoAddrPtr, 0);
    write_byte(LORARegPayloadLength, len);

    write_fifo(data, len);
    fprintf(stderr, "before TX IRQ=%x\n", read_byte(LORARegIrqFlags));
    // dump_dio();

    lora_set_opmode(OPMODE_TX); //start sending here

    do {
        state = digitalRead(PIN_DIO0);
    } while(state == 0);

    // dump_dio();

    flags = read_byte(LORARegIrqFlags);
    assert(flags & IRQ_LORA_TXDONE_MASK);
    write_byte(LORARegIrqFlagsMask, 0xFF);
    write_byte(LORARegIrqFlags, 0xFF);

    fprintf(stderr, "after TX IRQ=%x\n", read_byte(LORARegIrqFlags));

    piUnlock(COMMAND_LOCK_NUMBER);
    // fprintf(stderr, "Data sent\n");

    return 0;
}

// return the length of received packet
// 0 for timeout
int lora_rx_single(rx_info_t *data, int timeout_symbols)
{
    int state;
    uint8_t flags;
    uint8_t mode;

    piLock(COMMAND_LOCK_NUMBER);

    mode = lora_get_opmode();
    if (mode != OPMODE_STANDBY && mode != OPMODE_SLEEP)
    {
        fprintf(stderr, "Cannot switch to rx singal mode\n");
        piUnlock(COMMAND_LOCK_NUMBER);
        return -1;
    }
    lora_set_opmode(OPMODE_STANDBY);

    #ifdef CONFIG_LORA_IS_GATEWAY
        // do NOT invert I and Q when RX on gateway
        lora_clear_invert_iq();
    #else
        // invert I and Q when RX on mote
        lora_set_invert_iq();
    #endif

    if (timeout_symbols > 0)
    {
        lora_set_rx_timeout(timeout_symbols);
    }

    write_byte(RegLna, CONFIG_LORA_LNA_RX_GAIN);
    write_byte(LORARegPayloadMaxLength, CONFIG_LORA_MAX_RX_LENGTH);
    write_byte(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP);
    // clear flags
    write_byte(LORARegIrqFlags, 0xFF);
    write_byte(LORARegIrqFlagsMask, (uint8_t)(~(IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK|IRQ_LORA_CRCERR_MASK)));
    // start receiving
    lora_set_opmode(OPMODE_RX_SINGLE);

    // wait for rxdone or timeout
    do {
        state = digitalRead(PIN_DIO0) | digitalRead(PIN_DIO1);
    }
    while(state == 0);
    // check flags
    flags = read_byte(LORARegIrqFlags);
    // fill rx_info_t with bw, sf, freq, second, nanosecond
    fill_rx_info_t(data);
    if (flags & IRQ_LORA_RXDONE_MASK)
    {
        data->snr = lora_get_last_packet_snr();
        data->rssi = lora_get_last_packet_rssi();
        data->cr = lora_get_last_packet_coding_rate();
        // only copy data if crc check is OK or no crc checksum
        if ((lora_get_last_packet_crc() == 0) || ((flags & IRQ_LORA_CRCERR_MASK) == 0))
        {
            data->len = read_byte(LORARegRxNbBytes);
            // put fifo pointer to last packet
            write_byte(LORARegFifoAddrPtr, read_byte(LORARegFifoRxCurrentAddr));
            // copy to buffer
            read_fifo(data->buf, data->len);
        }
        else
        {
            fprintf(stderr, "CRC check failed.\n");
            data->len = 0;
        }
    }
    else
    {
        fprintf(stderr, "RX timeout.\n");
        data->len = 0;
    }
    write_byte(LORARegIrqFlagsMask, 0xFF);
    write_byte(LORARegIrqFlags, 0xFF);
    piUnlock(COMMAND_LOCK_NUMBER);
    return data->len;
}


static int volatile rx_running;

int lora_rx_continuous(rx_info_t *data)
{
    uint8_t mode;
    uint8_t flags;

    piLock(COMMAND_LOCK_NUMBER);
    mode = lora_get_opmode();
    if (mode != OPMODE_RX)
    {
        // only set mode to RX if not in RX mode
        if (mode != OPMODE_STANDBY && mode != OPMODE_SLEEP)
        {
            fprintf(stderr, "Cannot switch to rx mode\n");
            piUnlock(COMMAND_LOCK_NUMBER);
            return -1;
        }
        lora_set_opmode(OPMODE_STANDBY);

        #ifdef CONFIG_LORA_IS_GATEWAY
            // do NOT invert I and Q when RX on gateway
            lora_clear_invert_iq();
        #else
            // invert I and Q when RX on mote
            lora_set_invert_iq();
        #endif

        write_byte(RegLna, CONFIG_LORA_LNA_RX_GAIN);
        write_byte(LORARegPayloadMaxLength, CONFIG_LORA_MAX_RX_LENGTH);
        write_byte(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
        // clear flags
        write_byte(LORARegIrqFlags, 0xFF);
        write_byte(LORARegIrqFlagsMask, (uint8_t)(~(IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK)));
        // start receiving
        lora_set_opmode(OPMODE_RX);
    }

    rx_running = 1;
    fprintf(stderr, "RX continuous started.\n");

    // while not rxdone and rx_running==1
    while ((digitalRead(PIN_DIO0) == 0) && rx_running)
    {
        usleep(100);    //sleep 200us
    }
    // check flags
    flags = read_byte(LORARegIrqFlags);
    fprintf(stderr, "IRQ=%x\n", flags);
    // fill rx_info_t with bw, sf, freq, second, nanosecond
    fill_rx_info_t(data);
    if (running && (flags & IRQ_LORA_RXDONE_MASK))
    {
        // if rxdone
        write_byte(LORARegIrqFlags, 0xFF);

        data->snr = lora_get_last_packet_snr();
        data->rssi = lora_get_last_packet_rssi();
        data->cr = lora_get_last_packet_coding_rate();
        if ((lora_get_last_packet_crc() == 0) || ((flags & IRQ_LORA_CRCERR_MASK) == 0))
        {
            data->len = read_byte(LORARegRxNbBytes);
            // put fifo pointer to last packet
            write_byte(LORARegFifoAddrPtr, read_byte(LORARegFifoRxCurrentAddr));
            // copy to buffer
            read_fifo(data->buf, data->len);
        }
        else
        {
            data->len = 0;
            fprintf(stderr, "CRC failed.\n");
        }
        fprintf(stderr, "Got a packet of %d bytes.\n", data->len);
    }
    else
    {
        lora_set_opmode(OPMODE_STANDBY);
        write_byte(LORARegIrqFlags, 0xFF);
        write_byte(LORARegIrqFlagsMask, 0xFF);
        rx_running = 0;
        fprintf(stderr, "RX exited prematurely.\n");
        data->len = -1;
    }
    piUnlock(COMMAND_LOCK_NUMBER);
    return data->len;
}

int lora_rx_continuous_stop()
{
    rx_running = 0;
    piLock(COMMAND_LOCK_NUMBER);
    piUnlock(COMMAND_LOCK_NUMBER);
    return 0;
}

int lora_init()
{
    pin_init();

    piLock(COMMAND_LOCK_NUMBER);
    lora_reset();
    if (lora_get_version() == 0x12)
    {
        lora_set_opmode(OPMODE_SLEEP);

        // //calibrate
        // write_byte(RegPaConfig, 0);
        // write_byte(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
        // while((read_byte(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
        // disable all irqs
        write_byte(LORARegIrqFlags, 0xFF);
        write_byte(LORARegIrqFlagsMask, 0xFF);
    }
    else
    {
        return -1;
    }

    piUnlock(COMMAND_LOCK_NUMBER);
    return 0;
}


// always explicit mode, SF >= 7
int lora_config(int sf, int cr, int bw)
{
    uint8_t mc1 = 0;
    uint8_t mc2 = 0;
    uint8_t mc3 = 0;

    fprintf(stderr, "Set SF=%d, CR=%d, BW=%d, prelen=%d, sync=0x%x\n", sf, cr, bw, CONFIG_LORA_PREAMBLE_LENGTH, CONFIG_LORA_SYNC_WORD);
    switch (bw)
    {
        case 125: mc1 |= 0x70; break;
        case 250: mc1 |= 0x80; break;
        case 500: mc1 |= 0x90; break;
        default:
            // fprintf(stderr, "Unknown BW = %d\n", bw);
            return -1;
    }
    switch (cr) {
        case 45: mc1 |= 0x02; break;
        case 46: mc1 |= 0x04; break;
        case 47: mc1 |= 0x06; break;
        case 48: mc1 |= 0x08; break;
        default:
            // fprintf(stderr, "Unknown CR = %d\n", cr);
            return -1;
    }

#ifdef CONFIG_LORA_PAYLOAD_CRC
    mc2 |= SX1278_MC2_RX_PAYLOAD_CRCON;
#endif

    if (sf >= 7 && sf <= 12)
    {
        mc2 |= (sf << 4) & 0xf0;
    }
    else
    {
        // fprintf(stderr, "Unknown SF = %d\n", sf);
        return -1;
    }

    piLock(COMMAND_LOCK_NUMBER);
    // set mc1 and mc2
    write_byte(LORARegModemConfig1, mc1);
    write_byte(LORARegModemConfig2, mc2);
    // use auto AGC
    mc3 = SX1278_MC3_AGCAUTO;
    if ((sf == 11 || sf == 12) && bw == 125)
        mc3 |= SX1278_MC3_LOW_DATA_RATE_OPTIMIZE;
    write_byte(LORARegModemConfig3, mc3);

    // set preamble length
    write_byte(LORARegPreambleMsb, (CONFIG_LORA_PREAMBLE_LENGTH >> 8) & 0xff);
    write_byte(LORARegPreambleLsb, (CONFIG_LORA_PREAMBLE_LENGTH) & 0xff);
    // set sync word
    write_byte(LORARegSyncWord, CONFIG_LORA_SYNC_WORD);
    piUnlock(COMMAND_LOCK_NUMBER);
    return 0;
}

void lora_set_txpower(int txpower)
{
    // use PA_HP, txpower in [2, 17] dBm range
    int8_t pw = txpower;
    if(pw >= 17) {
        pw = 17;
    } else if(pw < 2) {
        pw = 2;
    }
    pw -= 2;

    piLock(COMMAND_LOCK_NUMBER);
    // Pout = 17-(15-pw) = pw-2
    write_byte(RegPaConfig, (uint8_t)(0xF0 | (pw&0xf)));
    write_byte(RegPaDac, 0x87);
    // trip current = 200mA
    write_byte(RegOcp, 0x37);
    piUnlock(COMMAND_LOCK_NUMBER);
}

void lora_cleanup()
{
    // fprintf(stderr, "Cleaning up SX1278.\n");
    piLock(COMMAND_LOCK_NUMBER);
    lora_set_opmode(OPMODE_SLEEP);
    pin_cleanup();
    piLock(COMMAND_LOCK_NUMBER);
}

void lora_set_standby()
{
    piLock(COMMAND_LOCK_NUMBER);
    lora_set_opmode(OPMODE_STANDBY);
    piUnlock(COMMAND_LOCK_NUMBER);
    // delay(10);
}

void lora_set_sleep()
{
    piLock(COMMAND_LOCK_NUMBER);
    lora_set_opmode(OPMODE_SLEEP);
    piUnlock(COMMAND_LOCK_NUMBER);
}


void dump_hex(void *data, int len)
{
    uint8_t *ptr = data;
    int i;
    for (i = 0; i < len; i++)
    {
        printf("%02x", ptr[i]);
    }
    printf("\n");
}
