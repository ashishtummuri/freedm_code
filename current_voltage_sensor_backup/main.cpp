/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * This example uses OTAA to join the LoRaWAN network and then sends a
 * "hello world" uplink message periodically and prints out the
 * contents of any downlink message.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and OTAA settings 
#include "config.h"
#include "ssd1306.h"
#include "textRenderer/TextRenderer.h"
#include "hardware/i2c.h"

// edit with LoRaWAN Node Region and OTAA settings 
#include "config.h"
using namespace pico_ssd1306;

#define ADC_BITS 12
#define ADC_COUNTS (1 << ADC_BITS)
#define SUPPLY_VOLTAGE 3283
#define SUPPLY_VOLTAGE_SENSOR 5056

// pin configuration for SX1276 radio module
const struct lorawan_sx1276_settings sx1276_settings = {
    .spi = {
        .inst = PICO_DEFAULT_SPI_INSTANCE,
        .mosi = PICO_DEFAULT_SPI_TX_PIN,
        .miso = PICO_DEFAULT_SPI_RX_PIN,
        .sck  = PICO_DEFAULT_SPI_SCK_PIN,
        .nss  = 8
    },
    .reset = 9,
    .dio0  = 7,
    .dio1  = 10
};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui   = LORAWAN_DEVICE_EUI,
    .app_eui      = LORAWAN_APP_EUI,
    .app_key      = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

void current_voltage_init();
double current_get(uint num_samples, double current_calibration, double offset_current, double *current_samples);
double voltage_get(uint num_samples, double voltage_calibration, double offset_voltage, double *voltage_samples);
double calculate_power(double *voltage_samples, double *current_samples, uint num_samples);
double calculate_apparent_power(double voltage_rms, double current_rms);
double calculate_reactive_power(double apparent_power, double active_power);
double calculate_power_factor(double active_power, double apparent_power);

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    current_voltage_init();

    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }
    
    // printf("Pico LoRaWAN - Hello OTAA\n\n");

    // uncomment next line to enable debug

    // lorawan_debug(true);

    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_otaa(&sx1276_settings, LORAWAN_REGION, &otaa_settings) < 0) {
        printf("failed!!!\n");
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    // Start the join process and wait
    printf("Joining LoRaWAN network ... ");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process();
    }
    printf("joined successfully!\n");

    uint num_samples = 10000;
    double current_calibration =  51.61;
    double voltage_calibration =  1051.8;
    double offset_current = ADC_COUNTS >> 1;
    double offset_voltage = ADC_COUNTS >> 1;

    char current_str[16];
    char voltage_str[16];
    char power_str[16];

    double current_samples[num_samples];
    double voltage_samples[num_samples];
    double adc_current_rms;
    double adc_voltage_rms;
    double active_power;
    double apparent_power;
    double reactive_power;
    double power_factor;
    uint32_t last_message_time = 0;

    // loop forever
    while (1) {
        // let the lorwan library process pending events
        lorawan_process();
        adc_current_rms = current_get(num_samples, current_calibration, offset_current, current_samples);
        adc_voltage_rms = voltage_get(num_samples, voltage_calibration, offset_voltage, voltage_samples);
        active_power = calculate_power(voltage_samples, current_samples, num_samples);
        apparent_power = calculate_apparent_power(adc_voltage_rms, adc_current_rms);
        reactive_power = calculate_reactive_power(apparent_power, active_power);
        power_factor = calculate_power_factor(active_power, apparent_power);

        // get the current time and see if 5 seconds have passed
        // since the last message was sent
        char payload[100];  
        sprintf(payload, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",adc_current_rms, adc_voltage_rms, active_power, apparent_power, reactive_power, power_factor);        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - last_message_time) > 5000) {
            // try to send an unconfirmed uplink message
            printf("sending unconfirmed message '%s' ... ", payload);
            if (lorawan_send_unconfirmed(payload, strlen(payload), 2) < 0) {
                printf("failed!!!\n");
            } else {
                printf("success!\n");
            }

            last_message_time = now;
        }

        // check if a downlink message was received
        receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
        if (receive_length > -1) {
            printf("received a %d byte message on port %d: ", receive_length, receive_port);

            for (int i = 0; i < receive_length; i++) {
                printf("%02x", receive_buffer[i]);
            }
            printf("\n");
        }
    }

    return 0;
}

void current_voltage_init()
{
    adc_init();
    adc_gpio_init(27); // Use Pin 27 for ADC1
    adc_gpio_init(26); // Use Pin 26 for ADC0
}

double current_get(uint num_samples, double current_calibration, double offset_current, double *current_samples)
{
    adc_select_input(1);
    double sample_current = 0;
    double sum_current = 0;
    for (uint sample = 0; sample < num_samples; sample++) {
        sample_current = adc_read();
        offset_current += ((sample_current - offset_current) / 4096);
        double filtered_current = sample_current - offset_current;

        current_samples[sample] = filtered_current;  // Store the sample

        double sqrt_current = filtered_current * filtered_current;
        sum_current += sqrt_current;
    }
    double current_ratio = current_calibration * ((SUPPLY_VOLTAGE / 1000.0) / ADC_COUNTS);
    double current_rms = current_ratio * sqrt(sum_current / num_samples);
    return current_rms;
}

double voltage_get(uint num_samples, double voltage_calibration, double offset_voltage, double *voltage_samples) {
    adc_select_input(0);
    double sample_voltage;
    double sum_voltage = 0;
    for (uint sample = 0; sample < num_samples; sample++) {
        sample_voltage = adc_read();
        offset_voltage += ((sample_voltage - offset_voltage) / 4096);
        double filtered_voltage = sample_voltage - offset_voltage;

        voltage_samples[sample] = filtered_voltage;  // Store the sample

        double sqrt_voltage = filtered_voltage * filtered_voltage;
        sum_voltage += sqrt_voltage;
    }
    double voltage_ratio = voltage_calibration * ((SUPPLY_VOLTAGE / 1000.0) / ADC_COUNTS);
    double voltage_rms = voltage_ratio * sqrt(sum_voltage / num_samples);
    return voltage_rms;
}

double calculate_power(double *voltage_samples, double *current_samples, uint num_samples) {
    double sum_power = 0;
    for (uint sample = 0; sample < num_samples; sample++) {
        sum_power += voltage_samples[sample] * current_samples[sample];
    }
    return sum_power / num_samples;
}

double calculate_apparent_power(double voltage_rms, double current_rms) {
    return voltage_rms * current_rms;
}

double calculate_reactive_power(double apparent_power, double active_power) {
    return sqrt(pow(apparent_power, 2) - pow(active_power, 2));
}

double calculate_power_factor(double active_power, double apparent_power) {
    return active_power / apparent_power;
}
