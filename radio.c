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
"LORARegSyncWord", "N/A", "N/A", "N/A", "N/A", "N/A", "N/A", "RegDioMapping1", \
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

void lora_reset()
{
    digitalWrite(PIN_RST, HIGH);
    delay(100);
    digitalWrite(PIN_RST, LOW);
    delay(100);
}

uint8_t lora_get_version()
{
    return read_byte(RegVersion);
}

void lora_set_opmode(uint8_t opmode)
{
    write_byte(RegOpMode, opmode | OPMODE_LORA | OPMODE_LOWFREQON);
}

uint8_t lora_get_opmode()
{
    return read_byte(RegOpMode);
}

void lora_set_sync_word(uint8_t sync)
{
    write_byte(LORARegSyncWord, sync);
}

uint8_t lora_get_sync_word()
{
    return read_byte(LORARegSyncWord);
}

void lora_set_coding_rate(cr_t cr)
{
    uint8_t val;
    val = (read_byte(LORARegModemConfig1) & 0xf1) | (uint8_t)cr;
    write_byte(LORARegModemConfig1, val);
}

cr_t lora_get_coding_rate()
{
    return (cr_t)((read_byte(LORARegModemConfig1) >> 1) & 0x07);
}

void lora_set_bandwidth(bw_t bw)
{
    uint8_t val;
    val = (read_byte(LORARegModemConfig1) & 0x0f) | (uint8_t)bw;
    write_byte(LORARegModemConfig1, val);
}

bw_t lora_get_bandwidth()
{
    return (bw_t)((read_byte(LORARegModemConfig1) >> 4) & 0x0f);
}

void lora_set_spreading_factor(sf_t sf)
{
    uint8_t val;
    val = (read_byte(LORARegModemConfig2) & 0x0f) | (uint8_t)sf;
    write_byte(LORARegModemConfig2, val);
}

sf_t lora_get_spreading_factor()
{
    return (sf_t)((read_byte(LORARegModemConfig2) >> 4) & 0x0f);
}

void lora_set_rx_payload_crc(uint8_t on)
{
    uint8_t val;
    val = (read_byte(LORARegModemConfig2) & 0xfb) | (on << 2);
    write_byte(LORARegModemConfig2, val);
}

void lora_set_rx_timeout(int symbols)
{
    assert(symbols >= 0);
    assert(symbols <= 0x3ff);

    uint8_t val;
    val = (read_byte(LORARegModemConfig2) & 0xfc) | ((symbols >> 8) & 0x3);
    write_byte(LORARegModemConfig2, val);
    write_byte(LORARegSymbTimeoutLsb, (uint8_t)(val & 0xff));
}

int lora_get_rx_timeout()
{
    int msb;
    msb = (read_byte(LORARegModemConfig2) & 0x03) << 8;
    return msb | read_byte(LORARegSymbTimeoutLsb);
}

void lora_set_preamble_length(int length)
{
    write_byte(LORARegPreambleMsb, (length >> 8) & 0xff);
    write_byte(LORARegPreambleLsb, (length) & 0xff);
}

int lora_get_preamble_length()
{
    return ((int)read_byte(LORARegPreambleMsb) << 8) | read_byte(LORARegPreambleLsb);
}

void lora_set_payload_length(uint8_t length)
{
    assert(length > 0);
    write_byte(LORARegPayloadLength, length);
}

int lora_get_max_payload_length()
{
    return read_byte(LORARegPayloadMaxLength);
}

int lora_get_fifo_rx_offset()
{
    return read_byte(LORARegFifoRxByteAddr);
}

int lora_get_fifo_tx_base()
{
    return read_byte(LORARegFifoTxBaseAddr);
}

void lora_set_fifo_tx_base(uint8_t offset)
{
    write_byte(LORARegFifoTxBaseAddr, offset);
}

int lora_get_fifo_rx_base()
{
    return read_byte(LORARegFifoRxBaseAddr);
}

void lora_set_fifo_rx_base(uint8_t offset)
{
    write_byte(LORARegFifoRxBaseAddr, offset);
}

int lora_get_last_packet_address()
{
    return read_byte(LORARegFifoRxCurrentAddr);
}

int lora_get_last_packet_length()
{
    return read_byte(LORARegRxNbBytes);
}

int lora_get_last_packet_coding_rate()
{
    return (cr_t)((read_byte(LORARegModemStat) >> 5) & 0x07);
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

void lora_set_irq_mask(uint8_t mask)
{
    write_byte(LORARegIrqFlagsMask, mask);
}

uint8_t lora_get_irq_flags()
{
    return read_byte(LORARegIrqFlags);
}

void lora_clear_all_irq_flags()
{
    write_byte(LORARegIrqFlags, 0);
}


void lora_set_frequency(long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    write_byte(RegFrfMsb, (uint8_t)(frf>>16));
    write_byte(RegFrfMid, (uint8_t)(frf>>8));
    write_byte(RegFrfLsb, (uint8_t)(frf));
}

void lora_init()
{
    fprintf(stderr, "Init...\n");
    pin_init();
    lora_reset();
    assert(lora_get_version() == 0x12);

    fprintf(stderr, "Inited.");
}

void lora_cleanup()
{
    fprintf(stderr, "Cleaning up.\n");
    lora_set_opmode(OPMODE_SLEEP);
    pin_cleanup();
}
