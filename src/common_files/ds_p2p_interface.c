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
 * p2p_interface.c
 * Peer to peer interface communication.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"


#include "ds_p2p_interface.h"
#include "param_log_interface.h"
#include "eventtrigger.h"

#include "phryctoria.h"


#define THE_MAGIC_NUMBER 0xbc471117
#define P2P_PORT 5

EVENTTRIGGER(recvRoot, uint8, creater, uint8, time, uint8, type)
EVENTTRIGGER(recvToken, uint8, sender, uint8, owner, uint16, counter)
EVENTTRIGGER(sendToken, uint8, receiver, uint8, owner, uint16, counter)
EVENTTRIGGER(recvSatcut, uint8, process)

// NOTE: Phryctoria
// To sync time
static bool isCopterOn[NUM_AGENTS+1];
uint32_t syncMs;
// To log token and satcut
static Satcut satcutLog[MAX_AGENTS];
static int32_t timeSkew[MAX_AGENTS+1];
static uint16_t lastReceivedToken[NUM_AGENTS];
static Time lastReceivedRoot[NUM_AGENTS];
static Token lastSentToken[NUM_AGENTS];
static uint32_t lastSentTokenTime[NUM_AGENTS];
static uint32_t lastSentRootTime;
static Event lastSentRoot;

// State of peers
copter_full_state_t copters[MAX_ADDRESS];

// Higher level control
static uint8_t desiredFlyingCopters = INITIAL_DESIRED_FLYING_COPTERS;
static bool isControlDataSetYet = false;
static int32_t controlDataTimeMs = 0;  // Valid if isControlDataSetYet == true

static uint8_t counter = 0;

void initCopterOn(uint8_t copter_id) {
    isCopterOn[copter_id] = true;
    if (copter_id != 0)
        isCopterOn[0] = true;
}

bool allCopterOn() {
    for (int i=0; i<NUM_AGENTS+1; i++) {
        if (!isCopterOn[i]) {
            // DEBUG_PRINT("Copter %d is not ready\n", i);
            return false;
        }
    }
    // DEBUG_PRINT("All Copter On\n");
    return true;
}

bool allCopterFlyingAbove(float height, uint8_t copter_id) {
    bool ret = true;
    // except sniffer
    for (int i=1; i<NUM_AGENTS+1; i++) {
        if (i != copter_id) {
            if (copters[i].position.z < height) {
                // DEBUG_PRINT("copter %d: current height %.2f\n", i, (double)(copters[i].position.z));
                ret = false;
            }
        }
    }
    return ret;
}

bool allAgentsOff(uint8_t copter_id) {
    static bool ret = false;
    for (int i=0; i<NUM_AGENTS+1; i++) {
        if (i != copter_id) {
            ret |= isAlive(i);
        }
    }
    return (ret == false);
}

bool allAgentsLanding() {
    static bool landed[NUM_AGENTS+1];
    if (syncMs > 0) {
        for (int i=1; i<NUM_AGENTS+1; i++) {
            if (!landed[i]) {
                landed[i] = (copters[i].position.z < 0.2f);
            }
        }
        for (int i=1; i<NUM_AGENTS+1; i++) {
            if (!landed[i])
                return false;
        }
        return true; 
    }
    return false;
}

// NOTE: ICUAS24
copter_full_state_t getCopterFullState(uint8_t copter_id) {
    return copters[copter_id];
}
// end ICUAS24

uint8_t getCopterState(uint8_t copter_id){
    return copters[copter_id].state;
}

// for pilot(agents 0-N-1)
static void p2pcallbackHandler(P2PPacket *p) {
    // roots, tokens..
    if (p->port != P2P_PORT){
        DEBUG_PRINT("Wrong port %u\n", p->port);
        return;
    }

    uint32_t nowMs = T2M(xTaskGetTickCount());
    uint32_t nowMsSynced = nowMs - syncMs;

    switch((uint8_t)p->data[0])
    {
        case ROOT:
        {           
            static Event remote;
            memcpy(&remote, p->data+1, sizeof(Event));

            if (my_process != 7) {
                if (remote.t > lastReceivedRoot[remote.own_p]) {
                    lastReceivedRoot[remote.own_p] = remote.t;
                    remote.type = E_REMOTE;
                    remote.t += SKEW;
                    remote.ispos = isPosInt(remote.t);
                    remote.pvc[my_process] = remote.t;
                    list_append(&commA, &remote);
                }
            }
            eventTrigger_recvRoot_payload.time = remote.t;
            eventTrigger_recvRoot_payload.type = remote.type;
            eventTrigger_recvRoot_payload.creater = remote.own_p;
            eventTrigger(&eventTrigger_recvRoot);            
            break;
        }
        case TOKEN:
        {
            static Token rxToken;
            memcpy(&rxToken, p->data+1, sizeof(Token));
            
            if (my_process == rxToken.target_p) {
                // DEBUG_PRINT("P2P: Received Token:%u, %u\n", rxToken.own_p, rxToken.counter);
                if (rxToken.counter > lastReceivedToken[rxToken.own_p]) {
                    lastReceivedToken[rxToken.own_p] = rxToken.counter;
                    list_append_CommS(S_TOKEN, &rxToken);
                }
                else {
                    // DEBUG_PRINT("P2P: Duplicate Message. Ignore\n");
                }
            }                
            eventTrigger_recvToken_payload.owner = rxToken.own_p;
            eventTrigger_recvToken_payload.counter = rxToken.counter;
            eventTrigger_recvToken_payload.sender = rxToken.send_p;
            eventTrigger(&eventTrigger_recvToken);
            break;
        }
        case DATA:
        {
            static copter_message_t rxMessage;
            memcpy(&rxMessage, p->data+1, sizeof(rxMessage));

            if (rxMessage.magicNumber != THE_MAGIC_NUMBER) {
                DEBUG_PRINT("Wrong magic number %lu from %u\n", rxMessage.magicNumber, rxMessage.fullState.id);
                return;
            }

            uint8_t received_id = rxMessage.fullState.id;


            if (received_id >= MAX_ADDRESS) {
                DEBUG_PRINT("Can not handle id %u\n", received_id);
                return;
            }
            
            memcpy(&copters[received_id], &rxMessage.fullState, sizeof(copter_full_state_t));
            copters[received_id].timestamp = nowMs;
            copters[received_id].timestamp2_rx = nowMsSynced;

            timeSkew[received_id] = nowMsSynced - copters[received_id].timestamp2_tx;

            if (!isCopterOn[received_id]) {
                isCopterOn[received_id] = isCopterReady(received_id);
            }

            if (rxMessage.isControlDataValid) {
                int32_t newControlDataTimeMs = nowMs - rxMessage.ageOfControlDataMs;
                if ( ! isControlDataSetYet || newControlDataTimeMs > controlDataTimeMs) {
                    controlDataTimeMs = newControlDataTimeMs;
                    desiredFlyingCopters = rxMessage.desiredFlyingCopters;
                    isControlDataSetYet = true;
                }
            }

            // If not a message from the sniffer, send the position to the peer localization system to handle collision avoidance
            if (received_id > 0) {
                enum State state = rxMessage.fullState.state;
                if (state == STATE_TAKING_OFF ||
                    state == STATE_EXECUTING_TRAJECTORY ||
                    state == STATE_GOING_TO_PAD ||
                    state == STATE_GOING_TO_RANDOM_POINT ||
                    state == STATE_GOING_TO_TRAJECTORY_START ||
                    state == STATE_HOVERING ||
                    state == STATE_LANDING ||
                    state == STATE_PREPARING_FOR_LAND) {

                    positionMeasurement_t pos_measurement;
                    memcpy(&pos_measurement.pos, &rxMessage.fullState.position, sizeof(Position));

                    // DEBUG_PRINT("===================================================\n");
                    // DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d --> (%.2f , %.2f , %.2f)\n", rssi, received_id, counter,(double)pos_received.x,(double)pos_received.y,(double)pos_received.z);

                    pos_measurement.source =  MeasurementSourceLighthouse;
                    pos_measurement.stdDev = 0.01f;

                    peerLocalizationTellPosition(received_id, &pos_measurement);
                }
            }
            break;
        }
        case SATCUT:
        {
            static Satcut rxMessage;
            memcpy(&rxMessage, p->data+1, sizeof(Satcut));
            for (int i=0; i<NUM_AGENTS; i++) {
                satcutLog[rxMessage.send_p].pvc[i] = rxMessage.pvc[i];
            }
            satcutLog[rxMessage.send_p].type = rxMessage.type;
            eventTrigger_recvSatcut_payload.process = rxMessage.send_p;
            eventTrigger(&eventTrigger_recvSatcut);
            break;
        }
        default:
            break;    // do nothing
    }
}

void initP2P() {
    p2pRegisterCB(p2pcallbackHandler);
}

// broadcast data - for sniffer's recording
void broadcastToPeers(const copter_full_state_t* state, const uint32_t nowMs) {
    static P2PPacket packet;
    packet.data[0] = (uint8_t)DATA;
    static copter_message_t txMessage;

    memcpy(&txMessage.fullState, state, sizeof(txMessage.fullState));
    txMessage.fullState.counter = counter;

    txMessage.isControlDataValid = isControlDataSetYet;
    if (isControlDataSetYet) {
        txMessage.desiredFlyingCopters = desiredFlyingCopters;
        txMessage.ageOfControlDataMs = nowMs - controlDataTimeMs;
    }

    txMessage.magicNumber = THE_MAGIC_NUMBER;

    packet.port = P2P_PORT;
    memcpy(packet.data+1, &txMessage, sizeof(txMessage));

    packet.size = sizeof(txMessage) + sizeof(uint8_t);
    radiolinkSendP2PPacketBroadcast(&packet);

    counter += 1;
}

void broadcastRemoteRoot(Event *rightroot, int repeat) {
    // DEBUG_PRINT("Broadcast root\n");
    uint32_t nowms = T2M(xTaskGetTickCount());
    lastSentRootTime = nowms;
    memcpy(&lastSentRoot, rightroot, sizeof(Event));

    static uint32_t random_delay = 0;
    static P2PPacket packet;
    packet.data[0] = (uint8_t)ROOT;
    packet.port = P2P_PORT;

    memcpy(packet.data+1, rightroot, sizeof(Event));
    packet.size = sizeof(Event) + sizeof(uint8_t);

    for (int i=0; i<SEND_ROOT_REPEAT; i++) {
        radiolinkSendP2PPacketBroadcast(&packet);
        random_delay = rand() % (SEND_DELAY_MAX - SEND_DELAY_MIN) + SEND_DELAY_MIN;
        vTaskDelay(M2T(random_delay));
    }
    counter += 1;

    if (repeat < RESEND_EVENT_TIMES) {
        if ((nowms - lastSentRootTime) > RESEND_EVENT_TIMEOUT && lastSentRootTime != 0) {
            static Event resend;
            memcpy(&resend, &lastSentRoot, sizeof(Event));
            broadcastRemoteRoot(&resend, repeat+1);

        }
    }
}

void broadcastToken(Token *token, int repeat) {
    // DEBUG_PRINT("COMM: Broadcast: Token, ");
    // printToken(token);
    // record the token and time sent by this process
    uint32_t nowms = T2M(xTaskGetTickCount());
    lastSentTokenTime[token->own_p] = nowms;
    memcpy(&lastSentToken[token->own_p], token, sizeof(Token));

    static P2PPacket packet;
    packet.data[0] = (uint8_t)TOKEN;
    packet.port = P2P_PORT;
    
    memcpy(packet.data+1, token, sizeof(Token));

    packet.size = sizeof(Token) + sizeof(uint8_t);
    
    eventTrigger_sendToken_payload.counter = token->counter;
    eventTrigger_sendToken_payload.owner = token->own_p;
    eventTrigger_sendToken_payload.receiver = token->target_p;
    eventTrigger(&eventTrigger_sendToken);

    for (int i=0; i<SEND_TOKEN_REPEAT; i++) {
        uint32_t random_delay = rand() % (SEND_DELAY_MAX - SEND_DELAY_MIN) + SEND_DELAY_MIN;
        vTaskDelay(M2T(random_delay));
        radiolinkSendP2PPacketBroadcast(&packet);
    }

    // Resend token if the process does not send token i for a while
    if (repeat < RESEND_TOKEN_TIMES) {
        for (int i=0; i<NUM_AGENTS; i++) {
            if ((nowms - lastSentTokenTime[i]) > RESEND_TOKEN_TIMEOUT && lastSentTokenTime[i] != 0) {
                // DEBUG_PRINT("P2P: Resend Token %d\n", i);
                static Token resend_tk;
                memcpy(&resend_tk, &lastSentToken[i], sizeof(Token));
                broadcastToken(&resend_tk, repeat+1);
            }
        }
    }
    counter += 1;
}

void broadcastSatCut(Satcut *s) {
    static P2PPacket packet;
    packet.port = P2P_PORT;
    packet.data[0] = (uint8_t)SATCUT;

    memcpy(packet.data+1, s, sizeof(Satcut));
    packet.size = sizeof(Satcut) + sizeof(uint8_t);

    radiolinkSendP2PPacketBroadcast(&packet);

    counter += 1;
}

void initOtherStates(){
    for(int i=0;i<MAX_ADDRESS;i++){
        copters[i].state = STATE_UNKNOWN;
    }
}

bool isAlive(uint8_t copter_id) {
    uint32_t nowMs = T2M(xTaskGetTickCount());
    uint32_t dt = nowMs - copters[copter_id].timestamp;
    return dt < ALIVE_TIMEOUT_MS;
}

uint8_t compressVoltage(float voltage){
    if (voltage <= VOLTAGE_MIN) {
        return 0;
    }

    if (voltage >= VOLTAGE_MAX) {
        return 255;
    }

    return (uint8_t) ((voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 255);
}

float decompressVoltage(uint8_t voltage){
    return (voltage / 255.0f) * (VOLTAGE_MAX - VOLTAGE_MIN) + VOLTAGE_MIN;
}

void printOtherCopters(void){
    for (int i = 0; i < MAX_ADDRESS; i++) {
        if (copters[i].state != STATE_UNKNOWN){
            if (!peerLocalizationIsIDActive(i)){
                // DEBUG_PRINT("Copter %d is not active\n",i);
            }else{
                peerLocalizationOtherPosition_t *pos = peerLocalizationGetPositionByID(i);
                DEBUG_PRINT("Copter %d : %.2f , %.2f , %.2f --> %d with latest counter %d \n",i,(double)pos->pos.x,(double)pos->pos.y,(double)pos->pos.z,copters[i].state,copters[i].counter);
            }
        }
    }
}

static bool isFlyingState(enum State state) {
    return state > STATE_PREPARING_FOR_TAKE_OFF && state < STATE_GOING_TO_PAD;
}

static bool isReadyState(enum State state) {
    return (state >= STATE_IDLE && state <= STATE_WAIT_FOR_TAKE_OFF) || (state == STATE_SNIFFING);
}


bool isCopterFlying(uint8_t copter_id){
    return isAlive(copter_id) && isFlyingState(copters[copter_id].state);
}

bool isCopterReady(uint8_t copter_id) {
    return isAlive(copter_id) && isReadyState(copters[copter_id].state);
}

uint8_t getMinimumFlyingCopterId(void){
    uint8_t min_id = 11;
    for(int i = 1; i < MAX_ADDRESS; i++){
        if (isCopterFlying(i) && copters[i].id < min_id){
            min_id = copters[i].id;
        }
    }

    return min_id;
}

bool isAnyOtherCopterExecutingTrajectory(void){
    for(int i = 1; i < MAX_ADDRESS; i++){
        bool trajectory_condition = copters[i].state == STATE_GOING_TO_TRAJECTORY_START ||
                                    copters[i].state == STATE_EXECUTING_TRAJECTORY;

        if (isCopterFlying(i) && trajectory_condition){
            return true;
        }
    }
    return false;
}

static int getNrOfFlyingCopters(enum State ownState) {
    uint8_t flying_copters = 0;

    for (int i = 1; i < MAX_ADDRESS; i++) {
        if (isCopterFlying(i)) {
            flying_copters ++;
        }
    }

    if (isFlyingState(ownState)){
        flying_copters += 1;
    }

    return flying_copters;
}

bool needMoreCopters(enum State ownState) {
    return getNrOfFlyingCopters(ownState) < desiredFlyingCopters;
}

bool needLessCopters(enum State ownState){
    return getNrOfFlyingCopters(ownState) > desiredFlyingCopters;
}

uint8_t getDesiredFlyingCopters() {
    return desiredFlyingCopters;
}

void setDesiredFlyingCopters(uint8_t desired) {
    desiredFlyingCopters = desired;
    isControlDataSetYet = true;
    controlDataTimeMs = T2M(xTaskGetTickCount());
}

//LOGS

#define add_copter_log(i)   LOG_GROUP_START(id_##i)\
                            LOG_ADD(LOG_UINT8, state, &copters[i].state)\
                            LOG_ADD(LOG_UINT8, voltage, &copters[i].battery_voltage)\
                            LOG_ADD(LOG_UINT8, counter, &copters[i].counter)\
                            LOG_ADD(LOG_UINT32, timestamp, &copters[i].timestamp)\
                            LOG_ADD(LOG_UINT32, timestamp2_tx, &copters[i].timestamp2_tx)\
                            LOG_ADD(LOG_UINT32, timestamp2_rx, &copters[i].timestamp2_rx)\
                            LOG_ADD(LOG_FLOAT, pos_x, &copters[i].position.x)\
                            LOG_ADD(LOG_FLOAT, pos_y, &copters[i].position.y)\
                            LOG_ADD(LOG_FLOAT, pos_z, &copters[i].position.z)\
                            LOG_GROUP_STOP(id_i)

add_copter_log(1)
add_copter_log(2)
add_copter_log(3)
add_copter_log(4)
add_copter_log(5)
add_copter_log(6)
add_copter_log(7)
add_copter_log(8)
add_copter_log(9)

LOG_GROUP_START(ds)
LOG_ADD(LOG_UINT8, desired, &desiredFlyingCopters)
LOG_GROUP_STOP(ds)

// NOTE phryctoria - log satcut, process based 
#define satcut_log(i)   LOG_GROUP_START(satcut_##i)\
                        LOG_ADD(LOG_UINT8, type, &satcutLog[i].type)\
                        LOG_ADD(LOG_UINT8, pvc0, &satcutLog[i].pvc[0])\
                        LOG_ADD(LOG_UINT8, pvc1, &satcutLog[i].pvc[1])\
                        LOG_ADD(LOG_UINT8, pvc2, &satcutLog[i].pvc[2])\
                        LOG_ADD(LOG_UINT8, pvc3, &satcutLog[i].pvc[3])\
                        LOG_ADD(LOG_UINT8, pvc4, &satcutLog[i].pvc[4])\
                        LOG_ADD(LOG_UINT8, pvc5, &satcutLog[i].pvc[5])\
                        LOG_ADD(LOG_UINT8, pvc6, &satcutLog[i].pvc[6])\
                        LOG_GROUP_STOP(satcut_i)

satcut_log(0)
satcut_log(1)
satcut_log(2)
satcut_log(3)
satcut_log(4)
satcut_log(5)
satcut_log(6)

// NOTE phryctoria - log root, process based 
#define root_log(i) LOG_GROUP_START(root_##i)\
                    LOG_ADD(LOG_UINT32, time_tx, &rootLog[i].timestamp_tx)\
                    LOG_ADD(LOG_UINT32, time_rx, &rootLog[i].timestamp_rx)\
                    LOG_ADD(LOG_UINT8, time, &rootLog[i].t)\
                    LOG_ADD(LOG_UINT32, counter, &rootLog[i].counter)\
                    LOG_GROUP_STOP(root_i)

// root_log(0)
// root_log(1)
// root_log(2)
// root_log(3)
// root_log(4)
// root_log(5)
// root_log(6)

// NOTE phryctoria - log token, process based 
#define token_log(i) LOG_GROUP_START(token_##i)\
                    LOG_ADD(LOG_UINT32, time_tx, &tokenLog[i].timestamp_tx)\
                    LOG_ADD(LOG_UINT32, time_rx, &tokenLog[i].timestamp_rx)\
                    LOG_ADD(LOG_UINT32, counter, &tokenLog[i].counter)\
                    LOG_GROUP_STOP(token_i)

// token_log(0)
// token_log(1)
// token_log(2)
// token_log(3)
// token_log(4)
// token_log(5)
// token_log(6)

// id base: 0: sniffer, 1-7: agents 
LOG_GROUP_START(skew)
LOG_ADD(LOG_INT32, with0, &timeSkew[0])
LOG_ADD(LOG_INT32, with1, &timeSkew[1])
LOG_ADD(LOG_INT32, with2, &timeSkew[2])
LOG_ADD(LOG_INT32, with3, &timeSkew[3])
LOG_ADD(LOG_INT32, with4, &timeSkew[4])
LOG_ADD(LOG_INT32, with5, &timeSkew[5])
LOG_ADD(LOG_INT32, with6, &timeSkew[6])
LOG_ADD(LOG_INT32, with7, &timeSkew[7])
LOG_GROUP_STOP(skew)