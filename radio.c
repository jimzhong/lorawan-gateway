#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <pthread.h>

#include "radio.h"
#include "sx1278.h"

#define select_chip()   digitalWrite(pin_nss, LOW)
#define unselect_chip()    digitalWrite(pin_nss, HIGH)

#define cmd_lock()       pthread_mutex_lock(&lora_mutex)
#define cmd_unlock()     pthread_mutex_unlock(&lora_mutex)

#define lora_clear_irq_flags()    write_byte(LORARegIrqFlags, 0xFF)
#define lora_get_irq_flags()      read_byte(LORARegIrqFlags)

#define lora_set_invert_iq()      write_byte(LORARegInvertIQ, 0x27 | 0x40)
#define lora_clear_invert_iq()    write_byte(LORARegInvertIQ, 0x27 & 0xbf)

/* private global variables */

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


static int pin_nss;
static int pin_rst;
static int spi_ch;
static pthread_mutex_t lora_mutex = PTHREAD_MUTEX_INITIALIZER;
static radio_state_t lora_state;

/* end private global variables */


void static pin_init(int spi_ch, int spi_freq, int nss, int rst)
{
    wiringPiSetup();
    wiringPiSPISetup(spi_ch, spi_freq);
    digitalWrite(nss, HIGH);
    digitalWrite(rst, HIGH);
    pinMode(nss, OUTPUT);
    // pinMode(PIN_DIO0, INPUT);
    // pinMode(PIN_DIO1, INPUT);
    // pinMode(PIN_DIO2, INPUT);
    pinMode(rst, OUTPUT);

    pin_nss = nss;
    pin_rst = rst;
    spi_ch = spi_ch;
}

void static pin_cleanup()
{
    pinMode(pin_nss, INPUT);
    // pinMode(PIN_DIO0, INPUT);
    // pinMode(PIN_DIO1, INPUT);
    // pinMode(PIN_DIO2, INPUT);
    pinMode(pin_rst, INPUT);
}


uint8_t static read_byte(uint8_t addr)
{
    unsigned char spibuf[2];
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;

    select_chip();
    wiringPiSPIDataRW(spi_ch, spibuf, 2);
    unselect_chip();

    fprintf(stderr, "Read 0x%x from 0x%x\n", spibuf[1], addr);
    return spibuf[1];
}

void static write_byte(uint8_t addr, uint8_t value)
{
    unsigned char spibuf[2];
    spibuf[0] = addr | 0x80;
    spibuf[1] = value;

    select_chip();
    wiringPiSPIDataRW(spi_ch, spibuf, 2);
    unselect_chip();
    fprintf(stderr, "Wrote 0x%x to 0x%x\n", value, addr);
}

static void write_fifo (uint8_t* buf, uint8_t len)
{
    uint8_t addr = RegFifo | 0x80;

    select_chip();
    wiringPiSPIDataRW(spi_ch, &addr, 1);
    wiringPiSPIDataRW(spi_ch, buf, len);
    unselect_chip();
    // fprintf(stderr, "Wrote %u bytes to 0x%x\n", len, addr & 0x7f);
}

static void read_fifo (uint8_t* buf, uint8_t len)
{
    uint8_t addr = RegFifo & 0x7f;
    memset(buf, 0, sizeof(uint8_t) * len);

    select_chip();
    wiringPiSPIDataRW(spi_ch, &addr, 1);
    wiringPiSPIDataRW(spi_ch, buf, len);
    unselect_chip();

    // fprintf(stderr, "Read %u bytes from 0x%x\n", len, addr & 0x7f);
}

static void lora_reset()
{
    digitalWrite(pin_rst, LOW);
    delay(10);
    digitalWrite(pin_rst, HIGH);
    delay(50);
}

static uint8_t lora_get_version()
{
    uint8_t ver;
    ver = read_byte(RegVersion);
    return ver;
}

static uint8_t lora_get_opmode()
{
    uint8_t mode;
    mode = read_byte(RegOpMode) & OPMODE_MASK;
    return mode;
}


static void lora_set_opmode(uint8_t opmode)
{
    switch (opmode)
    {
        case OPMODE_SLEEP:
        case OPMODE_STANDBY:
            lora_state = RADIO_IDLE;
            break;
        case OPMODE_TX:
            lora_state = RADIO_TX_RUNNING;
            break;
        case OPMODE_RX:
        case OPMODE_RX_SINGLE:
            lora_state = RADIO_RX_RUNNING;
            break;
        case OPMODE_CAD:
            lora_state = RADIO_CAD;
            break;
    }
    opmode |= (OPMODE_LORA | OPMODE_LOWFREQON);
    write_byte(RegOpMode, opmode);
}


static void lora_set_rx_timeout(int symbols)
{
    assert(symbols >= 0);
    assert(symbols <= 0x3ff);

    uint8_t val;

    val = (read_byte(LORARegModemConfig2) & 0xfc) | ((symbols >> 8) & 0x3);
    write_byte(LORARegModemConfig2, val);
    write_byte(LORARegSymbTimeoutLsb, (uint8_t)(val & 0xff));
}

static int lora_get_last_packet_coding_rate()
{
    int cr;
    cr = (read_byte(LORARegModemStat) >> 5) & 0x07;
    switch (cr)
    {
        case 1: return 45;
        case 2: return 46;
        case 3: return 47;
        case 4: return 48;
    }
    return 0;
}

static int lora_get_last_packet_snr()
{
    return (int8_t)read_byte(LORARegPktSnrValue) >> 2;
}

static int lora_get_last_packet_rssi()
{
    int ret;
    ret = read_byte(LORARegPktRssiValue);
    return -164 + ret;
}

static int lora_get_last_packet_crc_on()
{
    return !!(read_byte(LORARegHopChannel) & 0x20);
}

static int lora_get_modem_status()
{
    return (read_byte(LORARegModemStat) & 0x1f);
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
    data->freq = (frf * 32000000) >> 19;

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

static int lora_set_sync_word(uint8_t sw)
{
    if (lora_state == RADIO_IDLE)
    {
        write_byte(LORARegSyncWord, sw);
        return 0;
    }
    return -1;
}

static int lora_set_preamble_len(int prelen)
{
    if (lora_state == RADIO_IDLE)
    {
        write_byte(LORARegPreambleMsb, (prelen >> 8) & 0xff);
        write_byte(LORARegPreambleLsb, (prelen) & 0xff);
        return 0;
    }
    return -1;
}

static int lora_set_txpower(int txpower)
{
    // use PA_HP, txpower in [2, 17] dBm range
    int8_t pw = txpower;
    if (lora_state != RADIO_IDLE)
    {
        return -1;
    }
    if(pw >= 17)
    {
        pw = 17;
    }
    else if(pw < 2)
    {
        pw = 2;
    }
    pw -= 2;

    // Pout = 17-(15-pw) = pw-2
    write_byte(RegPaConfig, (uint8_t)(0xF0 | (pw & 0xf)));
    write_byte(RegPaDac, 0x87);
    // trip current = 200mA
    write_byte(RegOcp, 0x37);
    return 0;
}

/* Below are exported functions */

int lora_get_current_rssi()
{
    int rssi;
    cmd_lock();
    rssi = -164 + read_byte(LORARegRssiValue);
    cmd_unlock();
    return rssi;
}

int lora_set_frequency(long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    if (freq < 450000000 && freq > 420000000)
    {
        cmd_lock();
        write_byte(RegFrfMsb, (uint8_t)(frf>>16));
        write_byte(RegFrfMid, (uint8_t)(frf>>8));
        write_byte(RegFrfLsb, (uint8_t)(frf));
        cmd_unlock();
        return 0;
    }
    return -1;
}

long lora_get_frequency()
{
    long long frf = 0;

    cmd_lock();
    frf = read_byte(RegFrfLsb);
    frf |= ((long)read_byte(RegFrfMid) << 8);
    frf |= ((long)read_byte(RegFrfMsb) << 16);
    cmd_unlock();
    frf = (frf * 32000000) >> 19;
    return (long)frf;
}

/* send data of len */
/* !!! data will be modified after SPI transaction !! */
int lora_tx(uint8_t *data, uint8_t len, int invert_iq)
{
    // fprintf(stderr, "Sending data of %d bytes.\n", len);
    cmd_lock();
    lora_set_opmode(OPMODE_STANDBY);

    if (invert_iq)
        lora_set_invert_iq();
    else
        lora_clear_invert_iq();

    // write_byte(RegDioMapping1, MAP_DIO1_LORA_NOP|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    lora_clear_irq_flags();
    write_byte(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    write_byte(LORARegFifoTxBaseAddr, 0);
    write_byte(LORARegFifoAddrPtr, 0);
    write_byte(LORARegPayloadLength, len);

    write_fifo(data, len);

    lora_set_opmode(OPMODE_TX); //start sending here

    while((lora_get_irq_flags() & IRQ_LORA_TXDONE_MASK) == 0)
    {
        delay(1);   // wait until TX done IRQ
    }

    lora_state = RADIO_IDLE;
    write_byte(LORARegIrqFlagsMask, 0xFF);
    lora_clear_irq_flags();

    cmd_unlock();
    return 0;
}

// return 0 on success
// -1 on timeouts and other failures
int lora_rx_single(rx_info_t *data, int timeout_symbols, int invert_iq)
{
    int retval;
    uint8_t flags;

    cmd_lock();

    if (lora_state != RADIO_IDLE)
    {
        cmd_unlock();
        return -1;
    }
    lora_set_opmode(OPMODE_STANDBY);

    if (invert_iq)
        lora_set_invert_iq();
    else
        lora_clear_invert_iq();

    if (timeout_symbols > 0)
    {
        lora_set_rx_timeout(timeout_symbols);
    }

    write_byte(RegLna, LORA_LNA_RX_GAIN_DEFAULT);
    write_byte(LORARegPayloadMaxLength, LORA_PAYLOAD_MAX_LENGTH_DEFAULT);
    // write_byte(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP);
    // clear flags
    lora_clear_irq_flags();
    write_byte(LORARegIrqFlagsMask, (uint8_t)(~(IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK|IRQ_LORA_CRCERR_MASK)));
    // start receiving
    lora_set_opmode(OPMODE_RX_SINGLE);

    // wait for rxdone or timeout flag is set
    do {
        delay(1);
        flags = lora_get_irq_flags();
    } while ((flags & (IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK)) == 0);
    // check flags
    flags = lora_get_irq_flags();
    // fill rx_info_t with bw, sf, freq, second, nanosecond

    fill_rx_info_t(data);

    if (flags & IRQ_LORA_RXDONE_MASK)
    {
        data->snr = lora_get_last_packet_snr();
        data->rssi = lora_get_last_packet_rssi();
        data->cr = lora_get_last_packet_coding_rate();
        // only copy data if crc check is OK or no crc checksum
        if ((lora_get_last_packet_crc_on() == 0) || ((flags & IRQ_LORA_CRCERR_MASK) == 0))
        {
            data->len = read_byte(LORARegRxNbBytes);
            // put fifo pointer to last packet
            write_byte(LORARegFifoAddrPtr, read_byte(LORARegFifoRxCurrentAddr));
            // copy to buffer
            read_fifo(data->buf, data->len);
            retval = 0;
        }
        else
        {
            //CRC check failed
            data->len = 0;
            retval = -1;
        }
    }
    else
    {
        // RX timed out
        data->len = 0;
        retval = -1;
    }
    write_byte(LORARegIrqFlagsMask, 0xFF);
    lora_clear_irq_flags();
    cmd_unlock();
    return retval;
}


/* Do continuous RX
The callback function is called on successful rx
*/
int lora_rx_continuous(void (*callback)(rx_info_t data), int invert_iq)
{
    uint8_t flags;
    rx_info_t data;
    int retval = 0;

    cmd_lock();
    if (lora_state != RADIO_IDLE)
    {
        cmd_unlock();
        return -1;
    }

    lora_set_opmode(OPMODE_STANDBY);

    if (invert_iq)
        lora_set_invert_iq();
    else
        lora_clear_invert_iq();

    write_byte(RegLna, LORA_LNA_RX_GAIN_DEFAULT);
    write_byte(LORARegPayloadMaxLength, LORA_PAYLOAD_MAX_LENGTH_DEFAULT);
    // clear flags
    lora_clear_irq_flags();
    write_byte(LORARegIrqFlagsMask, (uint8_t)(~(IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK)));
    // start receiving
    lora_set_opmode(OPMODE_RX);

    // printf("state = %d\n", lora_state);

    while (lora_state == RADIO_RX_RUNNING)
    {
        // while not rxdone and rx_running==1
        while ((lora_get_irq_flags() & IRQ_LORA_RXDONE_MASK) == 0)
        {
            printf("waiting for rxdone\n");
            delay(1);
            if (lora_state != RADIO_RX_RUNNING)
                break;
        }
        if (lora_state != RADIO_RX_RUNNING)
        {
            break;
        }
        // check flags
        flags = lora_get_irq_flags();
        lora_clear_irq_flags();

        printf("flags = 0x%x\n", flags);

        fill_rx_info_t(&data);

        if (flags & IRQ_LORA_RXDONE_MASK)
        {
            // if rxdone
            data.snr = lora_get_last_packet_snr();
            data.rssi = lora_get_last_packet_rssi();
            data.cr = lora_get_last_packet_coding_rate();

            if ((lora_get_last_packet_crc_on() == 0) || ((flags & IRQ_LORA_CRCERR_MASK) == 0))
            {
                data.len = read_byte(LORARegRxNbBytes);
                // put fifo pointer to last packet
                write_byte(LORARegFifoAddrPtr, read_byte(LORARegFifoRxCurrentAddr));
                // copy to buffer
                read_fifo(data.buf, data.len);
                callback(data);
            }
        }
        else
        {
            // lora_state != RADIO_RX_RUNNING
            // means lora_rx_continuous_stop is called
            lora_set_opmode(OPMODE_STANDBY);
            write_byte(LORARegIrqFlagsMask, 0xFF);
            retval = -1;
            break;
        }
    }
    cmd_unlock();
    return retval;
}

/* should only be call when rx continuous is running */
int lora_rx_continuous_stop()
{
    if (lora_state == RADIO_RX_RUNNING)
    {
        lora_state = RADIO_IDLE;
        cmd_lock();
        cmd_unlock();
        return 0;
    }
    return -1;
}

int lora_init(int spi_ch, int spi_freq, int nss, int rst)
{
    cmd_lock();
    pin_init(spi_ch, spi_freq, nss, rst);
    lora_reset();
    if (lora_get_version() == 0x12)
    {
        lora_set_opmode(OPMODE_STANDBY);
        // //calibrate
        // write_byte(RegPaConfig, 0);
        // write_byte(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
        // while((read_byte(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
        // disable all irqs
        lora_clear_irq_flags();
        write_byte(LORARegIrqFlagsMask, 0xFF);
        write_byte(RegDioMapping1, MAP_DIO1_LORA_NOP|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);

        lora_get_irq_flags();
    }
    else
    {
        cmd_unlock();
        return -1;
    }
    cmd_unlock();
    return 0;
}


// always explicit mode, SF >= 7
int lora_config(int sf, int cr, int bw, int txpower, int prelen, int syncword, uint8_t crcon)
{
    uint8_t mc1 = 0;
    uint8_t mc2 = 0;
    uint8_t mc3 = 0;

    // fprintf(stderr, "Set SF=%d, CR=%d, BW=%d, prelen=%d, sync=0x%x\n", sf, cr, bw, prelen, sw);
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

    if (crcon)
        mc2 |= SX1278_MC2_RX_PAYLOAD_CRCON;

    if (sf >= 7 && sf <= 12)
    {
        mc2 |= (sf << 4) & 0xf0;
    }
    else
    {
        // fprintf(stderr, "Unknown SF = %d\n", sf);
        return -1;
    }

    // use auto AGC
    mc3 = SX1278_MC3_AGCAUTO;
    if ((sf == 11 || sf == 12) && bw == 125)
        mc3 |= SX1278_MC3_LOW_DATA_RATE_OPTIMIZE;

    cmd_lock();
    // set mc1 and mc2
    write_byte(LORARegModemConfig1, mc1);
    write_byte(LORARegModemConfig2, mc2);
    write_byte(LORARegModemConfig3, mc3);

    lora_set_txpower(txpower);
    lora_set_sync_word(syncword);
    lora_set_preamble_len(prelen);

    cmd_unlock();
    return 0;
}

void lora_cleanup()
{
    // fprintf(stderr, "Cleaning up SX1278.\n");
    cmd_lock();
    lora_set_opmode(OPMODE_SLEEP);
    pin_cleanup();
    cmd_unlock();
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
