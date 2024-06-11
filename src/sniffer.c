/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * sniffer.c - Sniffer for swarm and GUI communication
 */

#include "choose_app.h"
#ifdef BUILD_SNIFFER_APP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "radiolink.h"
#include "configblock.h"
#include "float.h"
#include "estimator_kalman.h"
#include "eventtrigger.h"

#define DEBUG_MODULE "P2P"

#include "ds_p2p_interface.h"
#include "param_log_interface.h"
#include "common.h"

#include "phryctoria.h"
// #include "generic_list.h"
#include "array_list.h"

EVENTTRIGGER(syncTime, uint32, syncms)

static uint8_t my_id;

static xTimerHandle broadcastTimer;
static uint32_t endBroadcastTime = 0;

// NOTE phryctoria
static bool clockReset = false;
static uint32_t stop_log_ms;

static void broadcastData(xTimerHandle timer) {
    uint32_t nowMs = T2M(xTaskGetTickCount());
    
    // if (allCopterOn() && !clockReset) {
    //     clockReset = true;
    //     resetClock();
    // }
    // Only once
    if (!clockReset && allCopterOn() && allCopterFlyingAbove((float)Z_PREDICATE, 0)) {
        DEBUG_PRINT("clock reset\n");
        clockReset = true;
        syncMs = nowMs;
        if (canLog()) {
            DEBUG_PRINT("Start logging!\n");
            setusdLogging(1);
        }
        eventTrigger_syncTime_payload.syncms = syncMs;
        eventTrigger(&eventTrigger_syncTime);
    }
    
    if (nowMs < endBroadcastTime) {
        copter_full_state_t fullState;

        fullState.id = 0;  // The sniffer
        // fullState.counter - set when transmitted
        fullState.state = STATE_SNIFFING;
        fullState.battery_voltage = 0;
        fullState.timestamp = nowMs;
        fullState.position.x = 0.0f;
        fullState.position.y = 0.0f;
        fullState.position.z = 0.0f;    
        fullState.timestamp2_tx = nowMs - syncMs;
        fullState.timestamp2_rx = 0;
        broadcastToPeers(&fullState, nowMs);
    }

    // NOTE: Phryctoria
    if (clockReset && allAgentsLanding()) {
        DEBUG_PRINT("All agents off, turn off logging\n");
        // turn off logging
        stop_log_ms = nowMs + LOG_OFF_DELAY;
        clockReset = false;
    }

    if (nowMs > stop_log_ms && stop_log_ms > 0) {
        setusdLogging(0);
    }
}

static void broadcastDesiredFlyingCopters(uint8_t desired) {
    setDesiredFlyingCopters(desired);
    endBroadcastTime = T2M(xTaskGetTickCount()) + 1000;
}

static uint8_t lessCoptersVal;
static void lessCopters() {
    uint8_t desired = getDesiredFlyingCopters();
    if (desired > 0) {
        desired -= 1;
    }
    broadcastDesiredFlyingCopters(desired);
}

static uint8_t moreCoptersVal;
static void moreCopters() {
    broadcastDesiredFlyingCopters(getDesiredFlyingCopters() + 1);
}

void appMain()
{
    DEBUG_PRINT("Running Decentralized swarm sniffer ...\n");

    my_id = (uint8_t)0;

    initP2P();
    initOtherStates();
    // NOTE: phryctoria
    initParamLogInterfaceSniffer();

    // NOTE: phryctoria init.
    initPhryctoriaSniffer();
    initCopterOn(my_id);

    broadcastTimer = xTimerCreate("SendPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(broadcastTimer, 20);

    while(1) {
        vTaskDelay(M2T(SNIFFER_PRINT_PERIOD_MS));
        printOtherCopters();
    }
}

PARAM_GROUP_START(app)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, more, &moreCoptersVal, &moreCopters)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, less, &lessCoptersVal, &lessCopters)
PARAM_GROUP_STOP(app)


#endif // BUILD_SNIFFER_APP
