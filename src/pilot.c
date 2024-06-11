/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *m
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pilot.c - App for each copter of the Decentralized Swarm
 */

#include "choose_app.h"
#ifdef BUILD_PILOT_APP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// #include <semaphore.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "radiolink.h"
#include "configblock.h"
#include "log.h"
#include "float.h"
#include "estimator_kalman.h"
#include "ledseq.h"
#include "timers.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "pm.h"
#include "supervisor.h"
#include "settings.h"
#include "ds_p2p_interface.h"
#include "positions.h"
#include "common.h"
#include "param_log_interface.h"
#include "movement.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"
#include "eventtrigger.h"
#include "usec_time.h"

#include "phryctoria.h"
#include "array_list.h"

// Log
EVENTTRIGGER(syncTime, uint32, syncMs)
EVENTTRIGGER(logToken, uint8, proc, uint8, com)
EVENTTRIGGER(logRoot, uint8, proc, uint8, type, uint8, time)
EVENTTRIGGER(abstractorRT, uint8, type, uint32, lower, uint32, upper)
EVENTTRIGGER(slicerRT, uint8, type, uint32, lower, uint32, upper)

static Token log_tk;
static uint32_t log_rt_cnt;
static uint32_t absRT_cnt;
static uint32_t sliRT_cnt;
static uint32_t stop_log_ms;

static uint8_t my_id;

static xTimerHandle sendPosTimer;
static xTimerHandle stateTransitionTimer;
// NOTE: Phryctoria - abstractor and slicer
static xTimerHandle abstractorTimer;
static xTimerHandle slicerTimer;
static xTimerHandle sendTokenTimer;

static bool isInit = false;

//the state of the copter
enum State state = STATE_IDLE;

//Landing to pad
static uint32_t stabilizeEndTime_ms;
static float landingTimeCheckCharge_ms;

// NOTE: track predicate
static bool predTrack = false;
static float pre_z, cur_z;
static bool clockReset = false;
static uint32_t missionNextStart;
static uint32_t missionNextEndTime;
static float zAbove, zBelow;



Position positions_to_go[]={
    [0].x = +1 , [0].y = +1 ,[0].z = TAKE_OFF_HEIGHT,
    [1].x = +1 , [1].y = -1 ,[1].z = TAKE_OFF_HEIGHT,
    [2].x = -1 , [2].y = +1 ,[2].z = TAKE_OFF_HEIGHT,
    [3].x = -1 , [3].y = -1 ,[3].z = TAKE_OFF_HEIGHT,
};

static Position my_pos;

static float previous[3];
static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static uint32_t now_ms = 0;
static uint32_t position_lock_start_time_ms = 0;
static uint32_t random_time_for_next_event_ms = 0;

static bool isCrashInitialized = false;

// LEDs Interface
ledseqStep_t seq_flashing_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(300)},

  {    0, LEDSEQ_LOOP},

};

ledseqContext_t seq_estim_stuck = {
  .sequence = seq_flashing_def,
  .led = LED_ESTIMATOR_STUCK,
};

ledseqContext_t seq_crash = {
  .sequence = seq_flashing_def,
  .led = LED_CRASH,
};

static uint32_t getLowerBits(uint64_t start, uint64_t end) {
    return (uint32_t) ((end - start) & 0xFFFFFFFF);
}

static uint32_t getUpperBits(uint64_t start, uint64_t end) {
    return (uint32_t) (((end - start) >> 32) & 0xFFFFFFFF);
}

uint32_t get_next_random_timeout(uint32_t now_ms){
    uint32_t extra = (rand() % (TAKE_OFF_TIME_MAX - TAKE_OFF_TIME_MIN)) + TAKE_OFF_TIME_MIN;
    uint32_t timeout = now_ms + extra;
    DEBUG_PRINT("Next random timeout dt: %lu \n", extra);
    return timeout;
}

uint32_t get_next_random_mission_end(uint32_t now_ms){
    uint32_t extra = (rand() % (MISSION_ON_MAX - MISSION_ON_MIN)) + MISSION_DURATION;
    uint32_t timeout = now_ms + extra;
    // DEBUG_PRINT("Next random timeout dt: %lu \n", extra);
    return timeout;
}

uint32_t get_next_random_mission_start(uint32_t now_ms){
    uint32_t extra = (rand() % (MISSION_OFF_MAX - MISSION_OFF_MIN)) + MISSION_OFF_MIN;
    uint32_t timeout = now_ms + extra;
    // DEBUG_PRINT("Next random timeout dt: %lu \n", extra);
    return timeout;
}

// timers
static void broadcastData(xTimerHandle timer) {
    // 1. broadcast data to sniffer for record
    uint32_t nowMs = T2M(xTaskGetTickCount());



    copter_full_state_t fullState;

    fullState.id = my_id;
    // fullState.counter - set when transmitted
    fullState.state = state;
    fullState.battery_voltage = compressVoltage(getVoltage());
    fullState.timestamp = nowMs;
    fullState.timestamp2_tx = nowMs - syncMs;
    fullState.timestamp2_rx = 0;
    fullState.position.x = getX();
    fullState.position.y = getY();
    fullState.position.z = getZ();
    broadcastToPeers(&fullState, nowMs);

    if (!clockReset && allCopterOn() && allCopterFlyingAbove((float)Z_PREDICATE, my_id)) {
        if (!PRED(fullState.position.z)) {
            syncMs = nowMs;
            DEBUG_PRINT("clock reset\n");
            pre_z = fullState.position.z;
            predTrack = true;
            DEBUG_PRINT("Start monitoring predicate\n");
            clockReset = true;
            eventTrigger_syncTime_payload.syncMs = syncMs;
            eventTrigger(&eventTrigger_syncTime);
        }
    }

    if (predTrack) {
        // Monitoring start
        cur_z = fullState.position.z;   
        if ((PRED(pre_z) + PRED(cur_z)) == 1) {
            // root found
            Time nowS = ms_to_s(nowMs - syncMs);
            static enum EType roottype;
            if (!PRED(pre_z))
                roottype = E_LEFT;
            else  
                roottype = E_RIGHT;
            // update positive intervals
            updateIntervals(roottype, nowS);
            // send this event to abstractor
            Event newevent = {.own_p=my_process, .ispos=E_POS, .type=roottype, .pvc={0}, .t=nowS};
            newevent.pvc[my_process] = nowS;
            // DEBUG_PRINT("LOCAL: ");
            // printEvent(&newevent);
            list_append(&commA, &newevent);
        }
        pre_z = cur_z;
    }
}
static void getPos(Position *p) {
    p->x = getX();
    p->y = getY();
    p->z = getZ();
}

static void startTakeOffSequence() {

    //take multiple samples for the pad position
    Position pad_sampler = {0.0f, 0.0f, 0.0f};

    for(uint8_t i = 0; i < NUMBER_OF_PAD_SAMPLES; i++){
        pad_sampler.x += getX();
        pad_sampler.y += getY();
        pad_sampler.z += getZ();
        vTaskDelay(50); // check if it interferes with the other tasks
    }
    MUL_VECTOR_3D_WITH_SCALAR(pad_sampler, 1.0f / NUMBER_OF_PAD_SAMPLES);

    padX = pad_sampler.x;
    padY = pad_sampler.y;
    padZ = pad_sampler.z;
    DEBUG_PRINT("Base position: (%f, %f, %f)\n", (double)padX, (double)padY, (double)padZ );
    zAbove = padZ + (float)TAKE_OFF_HEIGHT;
    zBelow = zAbove - (float)Z_CHANGE;

    DEBUG_PRINT("Taking off...\n");
    crtpCommanderHighLevelTakeoff(padZ + TAKE_OFF_HEIGHT, 1.0);
}

// NOTE: ICUAS24
// always to take the trajectory
static bool shouldFlySpecialTrajectory() {
    // int random_number = 0;

    // if (SPECIAL_TRAJ_PROBABILITY > 0.0f) {
    //     int special_traj_prob_length = (int)(1.0f / SPECIAL_TRAJ_PROBABILITY);
    //     random_number = rand() % special_traj_prob_length;
    //     // DEBUG_PRINT("special_traj_prob_length %i\n", special_traj_prob_length);
    //     // DEBUG_PRINT("Random number: %i\n", random_number);
    // }

    // uint8_t minimumFlyingCopterId = getMinimumFlyingCopterId();
    // bool noOneElseIsFlyingTrajectory = !isAnyOtherCopterExecutingTrajectory();

    // return (EXECUTE_TRAJ && (random_number == 0) && (my_id <= minimumFlyingCopterId) && noOneElseIsFlyingTrajectory);
    return true;
}

static void stateTransition(xTimerHandle timer){
    // In the following checks , sequence of checks is important

    if(supervisorIsTumbled()) {
        state = STATE_CRASHED;
    }
    else if (isBatLow() && (
        state == STATE_HOVERING ||
        state == STATE_GOING_TO_RANDOM_POINT ||
        state == STATE_EXECUTING_TRAJECTORY
    )) {
        DEBUG_PRINT("Battery low, landing\n");
        gotoChargingPad(padX, padY, padZ);
        state = STATE_GOING_TO_PAD;
    }

    now_ms = T2M( xTaskGetTickCount() );

    if (now_ms > stop_log_ms && stop_log_ms > 0) {
        setusdLogging(0);
    }

    switch(state) {
        case STATE_IDLE:
            DEBUG_PRINT ("Let's go! Waiting for position lock...\n");
            resetLockData();
            position_lock_start_time_ms = now_ms;
            state = STATE_WAIT_FOR_POSITION_LOCK;
            break;
        case STATE_WAIT_FOR_POSITION_LOCK:
            if (hasLock()) {
                DEBUG_PRINT("Position lock acquired, ready for take off..\n");
                
                state = STATE_WAIT_FOR_TAKE_OFF;
            }
            break;
        case STATE_WAIT_FOR_TAKE_OFF:        // This is the main state when not flying
            if (! chargedForTakeoff()){
                //do nothing, wait for the battery to be charged
            }
            else if (needMoreCopters(state)){
                random_time_for_next_event_ms = get_next_random_timeout(now_ms);
                DEBUG_PRINT("More copters needed, preparing for take off...\n");
                state = STATE_PREPARING_FOR_TAKE_OFF;
            }
            break;
        case STATE_PREPARING_FOR_TAKE_OFF:
            if (! needMoreCopters(state)) {
                DEBUG_PRINT("Don't need more copters after all, going back to wait state\n");
                state = STATE_WAIT_FOR_TAKE_OFF;
            }
            else if (now_ms > random_time_for_next_event_ms){
                DEBUG_PRINT("Taking off...\n");
                startTakeOffSequence();
                setusdLogging(1);
                state = STATE_TAKING_OFF;
            }
            break;
        case STATE_TAKING_OFF:
            if (crtpCommanderHighLevelIsTrajectoryFinished()) {
                DEBUG_PRINT("Hovering, waiting for command to start\n");
                // NOTE: phryctoria - turn off CA
                // enableCollisionAvoidance();
                state = STATE_HOVERING;
            }
            break;
        case STATE_HOVERING:
            if (needLessCopters(state)){
                DEBUG_PRINT("More copters than desired are flying while hovering, need to land\n");
                random_time_for_next_event_ms = get_next_random_timeout(now_ms);
                state = STATE_PREPARING_FOR_LAND;
            } else {
                // NOTE: always taking the trajectory
                if (shouldFlySpecialTrajectory()) {
                    // DEBUG_PRINT("Special trajectory\n");
                    // NOTE: for solo flight
                    // clockReset = true;
                    if (clockReset) {
                        // start mission
                        DEBUG_PRINT("start misssion\n");
                        missionNextStart = get_next_random_mission_start(now_ms);
                        state = STATE_GOING_TO_TRAJECTORY_START;
                    } 
                    else {
                        DEBUG_PRINT("not ready yet\n");
                        gotoNextWaypoint(padX, padY, zAbove, NO_YAW, 1.0);
                        state = STATE_GOING_TO_RANDOM_POINT;
                    }
                } else {
                    // NOTE: not taken
                    // PositionWithYaw new_pos = RANDOMIZATION_METHOD(&my_pos);
                    // DEBUG_PRINT("Normal new waypoint (%.2f, %.2f, %.2f)\n", (double)new_pos.x, (double)new_pos.y, (double)new_pos.z);
                    // gotoNextWaypoint(new_pos.x, new_pos.y, new_pos.z, new_pos.yaw, DELTA_DURATION);
                    DEBUG_PRINT("not ready yet\n");
                    gotoNextWaypoint(padX, padY, zAbove, NO_YAW, 1.0);
                    state = STATE_GOING_TO_RANDOM_POINT;
                }
            }
            break;
        case STATE_GOING_TO_TRAJECTORY_START:
            if (needLessCopters(state)){
                DEBUG_PRINT("More copters than desired are flying while hovering, need to land\n");
                random_time_for_next_event_ms = get_next_random_timeout(now_ms);
                state = STATE_PREPARING_FOR_LAND;
            }
            else {
                if (now_ms >= missionNextStart) {
                    // start mission
                    missionNextEndTime = get_next_random_mission_end(now_ms);
                    DEBUG_PRINT("Mission start\n");
                    gotoNextWaypoint(padX, padY, zBelow, NO_YAW, DELTA_DURATION);
                    state = STATE_EXECUTING_TRAJECTORY;
                }              
            }
            break;
        case STATE_EXECUTING_TRAJECTORY:        
            if (needLessCopters(state)){
                DEBUG_PRINT("More copters than desired are flying while hovering, need to land\n");
                random_time_for_next_event_ms = get_next_random_timeout(now_ms);
                state = STATE_PREPARING_FOR_LAND;
            }
            else {
                getPos(&my_pos);
                if (reachedNextWaypoint(my_pos)) {
                    if (now_ms >= missionNextEndTime) {
                        gotoNextWaypoint(padX, padY, zAbove, NO_YAW, DELTA_DURATION);
                        state = STATE_GOING_TO_RANDOM_POINT;
                    }
                    else {
                        gotoNextWaypoint(padX, padY, zBelow, NO_YAW, 1.0);
                        state = STATE_EXECUTING_TRAJECTORY;
                    }   
                }
            }
            break;
            // if (crtpCommanderHighLevelIsTrajectoryFinished()) {
            //     DEBUG_PRINT("Finished trajectory execution\n");
            //     state = STATE_PREPARING_FOR_LAND;
            // }
            // break;
        case STATE_GOING_TO_RANDOM_POINT:
            getPos(&my_pos);
            if (reachedNextWaypoint(my_pos)) {
                state = STATE_HOVERING;
            }
            break;
        case STATE_PREPARING_FOR_LAND:
            if (false) {
            // if (! needLessCopters(state)){ // another copter landed , no need to land after all
                DEBUG_PRINT("Another copter landed, no need to land finally\n");
                state = STATE_HOVERING;
            }
            else if (now_ms > random_time_for_next_event_ms){
                DEBUG_PRINT("Going to pad...\n");
                gotoChargingPad(padX, padY, padZ);
                state = STATE_GOING_TO_PAD;
            }
            break;
        case STATE_GOING_TO_PAD:
            if (reachedNextWaypoint(my_pos)) {
                DEBUG_PRINT("Over pad,starting lowering\n");
                disableCollisionAvoidance();
                crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, GO_TO_PAD_DURATION, false);
                stabilizeEndTime_ms = now_ms + STABILIZE_TIMEOUT;
                state = STATE_WAITING_AT_PAD;
            }
            break;
        case STATE_WAITING_AT_PAD:
            if (now_ms > stabilizeEndTime_ms || (
                (fabs(padX - getX()) < MAX_PAD_ERR) &&
                (fabs(padY - getY()) < MAX_PAD_ERR) &&
                (fabs((padZ + LANDING_HEIGHT) - getZ()) < MAX_PAD_ERR))) {
                if (now_ms > stabilizeEndTime_ms) {
                    DEBUG_PRINT("Warning: timeout!\n");
                }

                DEBUG_PRINT("Landing...\n");
                crtpCommanderHighLevelLand(padZ, LANDING_DURATION);
                state = STATE_LANDING;
            }
            break;
        case STATE_LANDING:
            if (crtpCommanderHighLevelIsTrajectoryFinished()) {
                if( outOfBounds(my_pos) ){
                    DEBUG_PRINT("Landed because of out of bounds, going to crashed state \n");
                    state = STATE_CRASHED;
                }
                else{
                    DEBUG_PRINT("Landed. Feed me!\n");
                    // NOTE: stop logging
                    crtpCommanderHighLevelStop();
                    landingTimeCheckCharge_ms = now_ms + 4000;
                    stop_log_ms = now_ms + LOG_OFF_DELAY;
                    state = STATE_CHECK_CHARGING;
                }
            }
            break;
        case STATE_CHECK_CHARGING:
            if (now_ms > landingTimeCheckCharge_ms) {
                DEBUG_PRINT("isCharging: %d\n", isCharging());
                state = STATE_WAIT_FOR_TAKE_OFF;
                // NOTE: deactivate repositioning
                // if (isCharging()) {
                //     state = STATE_WAIT_FOR_TAKE_OFF;
                // } else if (noCopterFlyingAbove()){
                //     DEBUG_PRINT("Not charging. Try to reposition on pad.\n");
                //     crtpCommanderHighLevelTakeoff(padZ + LANDING_HEIGHT + 0.1f , 1.0);
                //     state = STATE_REPOSITION_ON_PAD;
                // }
            }
            break;
        case STATE_REPOSITION_ON_PAD:
            if (crtpCommanderHighLevelIsTrajectoryFinished()) {
                DEBUG_PRINT("Over pad, stabilizing position\n");
                // NOTE: deactivate repositioning
                // gotoNextWaypoint(padX, padY, padZ + LANDING_HEIGHT, NO_YAW, 1.5);
                stabilizeEndTime_ms = now_ms + STABILIZE_TIMEOUT;
                state = STATE_WAITING_AT_PAD;
            }
            break;
        case STATE_CRASHED:
            if (! isCrashInitialized){
                crtpCommanderHighLevelStop();
                DEBUG_PRINT("Crashed, running crash sequence\n");
                ledseqRun(&seq_crash);
                isCrashInitialized = true;
                stop_log_ms = now_ms + LOG_OFF_DELAY;
            }
            break;

        default:
            break;
    }
}

static void abstractor(xTimerHandle timer)
{
    if (!list_isempty(&commA)) {
        static Event data;
        static uint64_t start_time, end_time;
        list_take(&commA, &data);

        eventTrigger_logRoot_payload.proc = data.own_p;
        eventTrigger_logRoot_payload.time = data.t;
        eventTrigger_logRoot_payload.type = data.type;
        log_rt_cnt++;
        eventTrigger(&eventTrigger_logRoot);

        if (data.type == E_REMOTE) {
            start_time = usecTimestamp();
            remoteRoot(&data);
            end_time = usecTimestamp();

            eventTrigger_abstractorRT_payload.type = REMOTEROOT;
        }
        else if (data.type == E_LEFT || data.type == E_RIGHT) {
            start_time = usecTimestamp();
            foundRoot(&data);
            end_time = usecTimestamp();

            eventTrigger_abstractorRT_payload.type = FOUNDROOT;
        }
        eventTrigger_abstractorRT_payload.lower = getLowerBits(start_time, end_time);
        eventTrigger_abstractorRT_payload.upper = getUpperBits(start_time, end_time);
        absRT_cnt++;
        eventTrigger(&eventTrigger_abstractorRT);
    }
}

static void slicer(xTimerHandle timer)
{
    if (!list_isempty(&commS)) {
        static CommS data;
        static uint64_t start_time, end_time;
        list_take(&commS, &data);
        if (data.type == S_EVENT) {
            start_time = usecTimestamp();
            receiveEvent(data.ev);
            end_time = usecTimestamp();

            eventTrigger_slicerRT_payload.type = RECVEVENT;
        }
        else {
            // data.type == S_TOKEN
            memcpy(&log_tk, &data.tk, sizeof(Token));
            eventTrigger_logToken_payload.proc = my_process;
            eventTrigger_logToken_payload.com = TOKEN_RECV;
            eventTrigger(&eventTrigger_logToken);

            start_time = usecTimestamp();
            receiveToken(data.tk);
            end_time = usecTimestamp();
            
            eventTrigger_slicerRT_payload.type = RECVTOKEN;
        }
        eventTrigger_slicerRT_payload.lower = getLowerBits(start_time, end_time);
        eventTrigger_slicerRT_payload.upper = getUpperBits(start_time, end_time);
        sliRT_cnt++;
        eventTrigger(&eventTrigger_slicerRT);
    }
}

static void broadcastTokens(xTimerHandle timer)
{
    if (!list_isempty(&sending_tokens)) {
        static Token txTok;
        list_take(&sending_tokens, &txTok);

        memcpy(&log_tk, &txTok, sizeof(Token));
        eventTrigger_logToken_payload.proc = my_process;
        eventTrigger_logToken_payload.com = TOKEN_SEND;
        eventTrigger(&eventTrigger_logToken);

        broadcastToken(&txTok, 0);            
    }
}

void appMain()
{
    if (isInit) {
        return;
    }

    uint64_t address = configblockGetRadioAddress();
    my_id = (uint8_t)((address) & 0x000000000f);   

    DEBUG_PRINT("Waiting for activation ...\n");
    // Get log and param ids
    initParamLogInterface();

    ledseqRegisterSequence(&seq_estim_stuck);
    ledseqRegisterSequence(&seq_crash);

    initP2P();
    initOtherStates();

    srand(my_id); // provide a unique seed for the random number generator

    initCollisionAvoidance();
    enableHighlevelCommander();
    defineTrajectory();

    previous[0] = 0.0f;
    previous[1] = 0.0f;
    previous[2] = 0.0f;

    // NOTE: Initialize phryctoria
    initPhryctoria(my_id);
    initCopterOn(my_id);

    reduceVelMax();
    usecTimerInit();

    
    // Track predicate
    // broadcast raw data to the sniffer
    sendPosTimer = xTimerCreate("SendPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(sendPosTimer, 20);

    // For flying
    stateTransitionTimer = xTimerCreate("AppTimer", M2T(CALC_NEXT_PERIOD_MS), pdTRUE, NULL, stateTransition);
    xTimerStart(stateTransitionTimer, 20);

    // abstractor
    abstractorTimer = xTimerCreate("AbstractorTimer", M2T(ABSTRACTOR_PERIOD_MS), pdTRUE, NULL, abstractor);
    xTimerStart(abstractorTimer, 70);

    // slicer
    slicerTimer = xTimerCreate("SlicerTimer", M2T(SLICER_PERIOD_MS), pdTRUE, NULL, slicer);
    xTimerStart(slicerTimer, 70);

    // queue for sending token(s)
    sendTokenTimer = xTimerCreate("SendTokenTimer", M2T(BROADCAST_TOKEN_PERIOD_MS), pdTRUE, NULL, broadcastTokens);
    xTimerStart(sendTokenTimer, 20);

    isInit = true;
}


LOG_GROUP_START(app)
  LOG_ADD(LOG_UINT8, state, &state)
LOG_GROUP_STOP(app)

LOG_GROUP_START(slicer_tk)
LOG_ADD(LOG_UINT8, own_p, &log_tk.own_p)
LOG_ADD(LOG_UINT8, flag, &log_tk.flag)
LOG_ADD(LOG_UINT8, type, &log_tk.type)
LOG_ADD(LOG_UINT8, eval, &log_tk.eval)
LOG_ADD(LOG_UINT16, counter, &log_tk.counter)
LOG_ADD(LOG_UINT8, target_p, &log_tk.target_p)
LOG_ADD(LOG_UINT8, target_t, &log_tk.target_t)
LOG_ADD(LOG_UINT8, cut0, &log_tk.cut[0])
LOG_ADD(LOG_UINT8, cut1, &log_tk.cut[1])
LOG_ADD(LOG_UINT8, cut2, &log_tk.cut[2])
LOG_ADD(LOG_UINT8, cut3, &log_tk.cut[3])
LOG_ADD(LOG_UINT8, cut4, &log_tk.cut[4])
LOG_ADD(LOG_UINT8, cut5, &log_tk.cut[5])
LOG_ADD(LOG_UINT8, cut6, &log_tk.cut[6])
LOG_ADD(LOG_UINT8, depend0, &log_tk.depend[0])
LOG_ADD(LOG_UINT8, depend1, &log_tk.depend[1])
LOG_ADD(LOG_UINT8, depend2, &log_tk.depend[2])
LOG_ADD(LOG_UINT8, depend3, &log_tk.depend[3])
LOG_ADD(LOG_UINT8, depend4, &log_tk.depend[4])
LOG_ADD(LOG_UINT8, depend5, &log_tk.depend[5])
LOG_ADD(LOG_UINT8, depend6, &log_tk.depend[6])
LOG_ADD(LOG_UINT8, ispos0, &log_tk.ispos[0])
LOG_ADD(LOG_UINT8, ispos1, &log_tk.ispos[1])
LOG_ADD(LOG_UINT8, ispos2, &log_tk.ispos[2])
LOG_ADD(LOG_UINT8, ispos3, &log_tk.ispos[3])
LOG_ADD(LOG_UINT8, ispos4, &log_tk.ispos[4])
LOG_ADD(LOG_UINT8, ispos5, &log_tk.ispos[5])
LOG_ADD(LOG_UINT8, ispos6, &log_tk.ispos[6])
LOG_ADD(LOG_UINT8, reset0, &log_tk.reset[0])
LOG_ADD(LOG_UINT8, reset1, &log_tk.reset[1])
LOG_ADD(LOG_UINT8, reset2, &log_tk.reset[2])
LOG_ADD(LOG_UINT8, reset3, &log_tk.reset[3])
LOG_ADD(LOG_UINT8, reset4, &log_tk.reset[4])
LOG_ADD(LOG_UINT8, reset5, &log_tk.reset[5])
LOG_ADD(LOG_UINT8, reset6, &log_tk.reset[6])
LOG_GROUP_STOP(slicer_tk)

LOG_GROUP_START(abs_root)
LOG_ADD(LOG_UINT32, cnt, &log_rt_cnt)
LOG_GROUP_STOP(abs_root)

LOG_GROUP_START(absRT)
LOG_ADD(LOG_UINT32, cnt, &absRT_cnt)
LOG_GROUP_STOP(absRT)

LOG_GROUP_START(sliRT)
LOG_ADD(LOG_UINT32, cnt, &sliRT_cnt)
LOG_GROUP_STOP(sliRT)

#endif // BUILD_PILOT_APP
