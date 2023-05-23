/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Can.h"
#include "pin_mux.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TX_MESSAGE_BUFFER_NUM      (8)

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool rxComplete = false;


int main(void)
{
    /* Initialize board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    // Inicilaizacion de componentes
    CAN_Init(&flexcanConfig);

    // Funcion de escritura
    Can_WriteTxMb(TX_MESSAGE_BUFFER_NUM, &mbConfig);

    //Funcion de lectura
    Can_MainFunction_Read();

    /* Waiting for Message receive finish. */
    while (!rxComplete)
    {
    }

    while (true)
    {
    }
}
