#ifndef __LORA_H__
#define __LORA_H__

void lora_setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
void lora_setFrequency(long frequency);
void lora_setSpreadingFactor(int sf);
void lora_setSignalBandwidth(long sbw);
void lora_setCodingRate4(int denominator);
void lora_setPreambleLength(long length);
void lora_setSyncWord(int sw);
void lora_crc(void);
void lora_noCrc(void);

int lora_available(void);
int lora_read(void);
int lora_peek(void);
void lora_flush(void);

void lora_receive(void);
void lora_idle(void);
void lora_sleep(void);

#endif
