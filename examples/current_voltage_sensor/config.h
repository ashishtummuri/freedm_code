/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

// LoRaWAN region to use, full list of regions can be found at: :
//   http://stackforce.github.io/LoRaMac-doc/LoRaMac-doc-v4.5.1/group___l_o_r_a_m_a_c.html#ga3b9d54f0355b51e85df8b33fd1757eec
#define LORAWAN_REGION          LORAMAC_REGION_US915

// LoRaWAN Device EUI (64-bit), NULL value will use Default Dev EUI
#define LORAWAN_DEVICE_EUI      "70B3D57ED006821A"
// end node 1- 70B3D57ED006821A
// end node 2 - 70B3D57ED006817E

// LoRaWAN Application / Join EUI (64-bit)
#define LORAWAN_APP_EUI         "F13DD0FA2619B115"
// end node 1 - F13DD0FA2619B115
// end node- 2 - F13DD0FA2619B104

// LoRaWAN Application Key (128-bit)
#define LORAWAN_APP_KEY         "F3694F135D1221C34020C67A3E455245"
// end node 1 - F3694F135D1221C34020C67A3E455245
// end node 2- 00DB4BDE1A6C1DD3ED9621E1C003AAD7

// LoRaWAN Channel Mask, NULL value will use the default channel mask 
// for the region
#define LORAWAN_CHANNEL_MASK    NULL
