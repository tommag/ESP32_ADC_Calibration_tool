/*
MIT License

Copyright (c) 2019 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_vfs_dev.h"
#include "esp_adc_cal.h"
#include "soc/efuse_reg.h"

#define ADC_NUM_SAMPLES (4096)
#define ADC1_CHANNEL (ADC_CHANNEL_6) //GPIO34 = ADC1 channel 6
#define ADC2_CHANNEL (ADC_CHANNEL_8) //GPIO25 = ADC2 channel 8
#define DEFAULT_VREF (1100) //Used for old/new calibration comparison

/* Following functions taken from esp_adc_cal_mod.c from https://esp32.com/viewtopic.php?t=8774#p36882 */
#define TP_LOW1_OFFSET                  278
#define TP_LOW2_OFFSET                  421
#define TP_LOW_MASK                     0x7F
#define TP_LOW_VOLTAGE                  150
#define TP_HIGH1_OFFSET                 3265
#define TP_HIGH2_OFFSET                 3406
#define TP_HIGH_MASK                    0x1FF
#define TP_HIGH_VOLTAGE                 850
#define TP_STEP_SIZE                    4

static const uint32_t adc1_tp_atten_scale[4] = {65504, 86975, 120389, 224310};
static const uint32_t adc2_tp_atten_scale[4] = {65467, 86861, 120416, 224708};
static const uint32_t adc1_tp_atten_offset[4] = {0, 1, 27, 54};
static const uint32_t adc2_tp_atten_offset[4] = {0, 9, 26, 66};

static inline int decode_bits(uint32_t bits, uint32_t mask, bool is_twos_compl)
{
    int ret;
    if (bits & (~(mask >> 1) & mask)) {      //Check sign bit (MSB of mask)
        //Negative
        if (is_twos_compl) {
            ret = -(((~bits) + 1) & (mask >> 1));   //2's complement
        } else {
            ret = -(bits & (mask >> 1));    //Sign-magnitude
        }
    } else {
        //Positive
        ret = bits & (mask >> 1);
    }
    return ret;
}

static uint32_t read_efuse_tp_low_custom(adc_unit_t adc_num, uint32_t efuse_val)
{
    //ADC reading at 150mV stored in two's complement format
    uint32_t ret;
    uint32_t bits;

    if (adc_num == ADC_UNIT_1) {
        ret = TP_LOW1_OFFSET;
        //bits = REG_GET_FIELD(TP_REG, EFUSE_RD_ADC1_TP_LOW);
        bits = (efuse_val >> EFUSE_RD_ADC1_TP_LOW_S) & EFUSE_RD_ADC1_TP_LOW_V;
    } else {
        ret = TP_LOW2_OFFSET;
        //bits = REG_GET_FIELD(TP_REG, EFUSE_RD_ADC2_TP_LOW);
        bits = (efuse_val >> EFUSE_RD_ADC2_TP_LOW_S) & EFUSE_RD_ADC2_TP_LOW_V;
    }
    ret += decode_bits(bits, TP_LOW_MASK, true) * TP_STEP_SIZE;
    return ret;     //Reading of ADC at 150mV
}

static uint32_t read_efuse_tp_high_custom(adc_unit_t adc_num, uint32_t efuse_val)
{
    //ADC reading at 850mV stored in two's complement format
    uint32_t ret;
    uint32_t bits;

    if (adc_num == ADC_UNIT_1) {
        ret = TP_HIGH1_OFFSET;
        //bits = REG_GET_FIELD(TP_REG, EFUSE_RD_ADC1_TP_HIGH);
        bits = (efuse_val >> EFUSE_RD_ADC1_TP_HIGH_S) & EFUSE_RD_ADC1_TP_HIGH_V;
    } else {
        ret = TP_HIGH2_OFFSET;
        //bits = REG_GET_FIELD(TP_REG, EFUSE_RD_ADC1_TP_HIGH);
        bits = (efuse_val >> EFUSE_RD_ADC1_TP_HIGH_S) & EFUSE_RD_ADC1_TP_HIGH_V;
    }
    ret += decode_bits(bits, TP_HIGH_MASK, true) * TP_STEP_SIZE;
    return ret;     //Reading of ADC at 850mV
}

static void characterize_using_two_point(adc_unit_t adc_num,
                                         adc_atten_t atten,
                                         uint32_t high,
                                         uint32_t low,
                                         uint32_t *coeff_a,
                                         uint32_t *coeff_b)
{
    const uint32_t *atten_scales;
    const uint32_t *atten_offsets;

    if (adc_num == ADC_UNIT_1) { //Using ADC 1
        atten_scales = adc1_tp_atten_scale;
        atten_offsets = adc1_tp_atten_offset;
    } else {    //Using ADC 2
        atten_scales = adc2_tp_atten_scale;
        atten_offsets = adc2_tp_atten_offset;
    }
    //Characterize ADC-Voltage curve as y = (coeff_a * x) + coeff_b
    uint32_t delta_x = high - low;
    uint32_t delta_v = TP_HIGH_VOLTAGE - TP_LOW_VOLTAGE;
    //Where coeff_a = (delta_v/delta_x) * atten_scale
    *coeff_a = (delta_v * atten_scales[atten] + (delta_x / 2)) / delta_x;   //+(delta_x/2) for rounding
    //Where coeff_b = high_v - ((delta_v/delta_x) * high_x) + atten_offset
    *coeff_b = TP_HIGH_VOLTAGE - ((delta_v * high + (delta_x / 2)) / delta_x) + atten_offsets[atten];
}

esp_adc_cal_value_t esp_adc_cal_characterize_custom(adc_unit_t adc_num,
                                             adc_atten_t atten,
                                             adc_bits_width_t bit_width,
                                             esp_adc_cal_characteristics_t *chars,
                                             uint32_t efuse_blk3)
{
    //Check parameters
    assert((adc_num == ADC_UNIT_1) || (adc_num == ADC_UNIT_2));
    assert(chars != NULL);
    assert(bit_width < ADC_WIDTH_MAX);

    esp_adc_cal_value_t ret;

    //Initialize most fields
    esp_adc_cal_characterize(adc_num, atten, bit_width, DEFAULT_VREF, chars);

    //Characterize based on Two Point values
    uint32_t high = read_efuse_tp_high_custom(adc_num, efuse_blk3);
    uint32_t low = read_efuse_tp_low_custom(adc_num, efuse_blk3);
    characterize_using_two_point(adc_num, atten, high, low, &chars->coeff_a, &chars->coeff_b);
    ret = ESP_ADC_CAL_VAL_EFUSE_TP;

    return ret;
}

static void read_both_adc(uint16_t *adc1_result, uint16_t *adc2_result)
{
    uint64_t adc1_acc = 0, adc2_acc = 0;

    for (int i = 0; i < ADC_NUM_SAMPLES; i++)
    {
        adc1_acc += adc1_get_raw((adc1_channel_t)ADC1_CHANNEL);
        int adc2_raw;
        adc2_get_raw((adc2_channel_t)ADC2_CHANNEL, ADC_WIDTH_BIT_12, &adc2_raw);
        adc2_acc += adc2_raw;
                
        // vTaskDelay(pdMS_TO_TICKS(1));
    }

    *adc1_result = (uint16_t)(adc1_acc / ADC_NUM_SAMPLES);
    *adc2_result = (uint16_t)(adc2_acc / ADC_NUM_SAMPLES);
}

static void flush_stdin()
{
    int ch;
    while ((ch = getchar()) != '\n' && ch != EOF);
}

static void calibrate_task()
{
    printf("To begin Two Point calibration, apply 150mV to GPIO 25 and GPIO34.\n");
    printf("Consider adding a 100nF decoupling capacitor to GND as close as possible to the GPIO.\n");
    printf("Press Enter when ready\n");
    getchar();
    flush_stdin();

    uint16_t adc1_150mV, adc2_150mV;
    read_both_adc(&adc1_150mV, &adc2_150mV);
    printf("\nFinal ADC readings after %d samples: %4hu, %4hu\n", ADC_NUM_SAMPLES, adc1_150mV, adc2_150mV);

    printf("\nNow, apply 850mV to GPIO 25 and GPIO34. Press Enter when ready.\n");
    getchar();
    flush_stdin();

    uint16_t adc1_850mV, adc2_850mV;
    read_both_adc(&adc1_850mV, &adc2_850mV);
    printf("\nFinal ADC readings after %d samples: %4hu, %4hu\n", ADC_NUM_SAMPLES, adc1_850mV, adc2_850mV);

    //Uncomment to test calculation with the forum post example. Should be 0xE778F207 at the end.
    // adc1_150mV = 306;
    // adc1_850mV = 3153;
    // adc2_150mV = 389;
    // adc2_850mV = 3206;

    /* Scaling and shifting as per https://esp32.com/viewtopic.php?t=8774#p36882 */
    int a1 = (adc1_150mV - TP_LOW1_OFFSET) / TP_STEP_SIZE;
    int b1 = (adc1_850mV - TP_HIGH1_OFFSET) / TP_STEP_SIZE;
    int a2 = (adc2_150mV - TP_LOW2_OFFSET) / TP_STEP_SIZE;
    int b2 = (adc2_850mV - TP_HIGH2_OFFSET) / TP_STEP_SIZE;

    printf("\nScaled and shifted calibration values: A1 = %d, B1 = %d, A2 = %d, B2 = %d", a1, b1, a2, b2);

    /* Modify two's complement to account for calibrated 0 value */
    if (a1 == 0) a1 = 0x40;
    if (a2 == 0) a2 = 0x40;
    if (b1 == 0) b1 = 0x100;
    if (b2 == 0) b2 = 0x100;

    /* Concatenate into a single 32 bit value */
    uint32_t efuse_blk3_value = (a1 & TP_LOW_MASK) |
        (b1 & TP_HIGH_MASK) << 7 |
        (a2 & TP_LOW_MASK) << 16 |
        (b2 & TP_HIGH_MASK) << 23;
    printf("\nEFUSE_BLK3_RDATA3 value: 0x%08X\n", efuse_blk3_value);

    printf("------------------------------\n");
    printf("To burn these calibration values permanently use the following commands:\n\n");
    printf("echo -n -e \\\\x%02x\\\\x%02x\\\\x%02x\\\\x%02x > efuse_blk3.bin\n",
         efuse_blk3_value & 0xFF,
        (efuse_blk3_value >> 8) & 0xFF,
        (efuse_blk3_value >> 16) & 0xFF,
        (efuse_blk3_value >> 24 & 0xFF));
    printf("espefuse.py burn_block_data --offset 12 BLK3 efuse_blk3.bin\n");
    printf("espefuse.py burn_efuse BLK3_PART_RESERVE 1\n\n");
    printf("------------------------------\n");



    /* Try the new values */
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_11db); 
    adc2_config_channel_atten(ADC2_CHANNEL, ADC_ATTEN_11db); 

    esp_adc_cal_characteristics_t adc1_old_chars, adc1_new_chars, adc2_old_chars, adc2_new_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, DEFAULT_VREF, &adc1_old_chars);
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_12Bit, DEFAULT_VREF, &adc2_old_chars);
    esp_adc_cal_characterize_custom(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, &adc1_new_chars, efuse_blk3_value);
    esp_adc_cal_characterize_custom(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_12Bit, &adc2_new_chars, efuse_blk3_value);
   
    printf("You can now change the voltage to test the new vs old calibration values.\nAttenuation is set to 11dB so you can measure voltages up to 3.3V.\n");
    printf("Current ADC reading: \nADC1\t\t\t\t\tADC2\nRaw\tOld cal mV\tNew cal mV\tRaw\tOld cal mV\tNew cal mV\n");
    while (1)
    {
        uint16_t adc1_val, adc2_val;
        read_both_adc(&adc1_val, &adc2_val);
        printf("\r%4u\t%4u\t\t%4u\t\t%4u\t%4u\t\t%4u", 
            adc1_val,
            esp_adc_cal_raw_to_voltage(adc1_val, &adc1_old_chars),
            esp_adc_cal_raw_to_voltage(adc1_val, &adc1_new_chars),
            adc2_val,
            esp_adc_cal_raw_to_voltage(adc2_val, &adc2_old_chars),
            esp_adc_cal_raw_to_voltage(adc2_val, &adc2_new_chars));
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void init_stdin()
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .use_ref_tick = true
    };
    ESP_ERROR_CHECK( uart_param_config(UART_NUM_0, &uart_config) );

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
            256, 0, 0, NULL, 0) );

    esp_vfs_dev_uart_use_driver(UART_NUM_0);
}

static void init_adc()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_0); 

    adc2_config_channel_atten(ADC2_CHANNEL, ADC_ATTEN_DB_0); 
}

void app_main()
{
    init_stdin();

    printf("Press Enter to begin.\n");
    getchar();
    flush_stdin();

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d\n", chip_info.revision);

    //Check TP is burned into eFuse
    printf("eFuse Two Point: %s\n",
        (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) ? "present" : "not present");

    //Check Vref is burned into eFuse
    printf("eFuse Vref: %s\n",
        (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) ?  "present" : "not present");

    init_adc();

    printf("Press Enter to begin calibration.\n");
    getchar();
    flush_stdin();

    xTaskCreate(calibrate_task, "calibrate_task", 2048, NULL, 10, NULL);
}