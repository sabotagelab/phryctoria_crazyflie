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
 * param_log_interface.c - Parameter log interface
 *
 */

#include "param_log_interface.h"

//App parameters

static bool CollisionAvoidanceEnabled = false;

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdStateEstimateVx;
static logVarId_t logIdStateEstimateVy;
static logVarId_t logIdStateEstimateVz;
static logVarId_t logIdKalmanVarPX;
static logVarId_t logIdKalmanVarPY;
static logVarId_t logIdKalmanVarPZ;
static logVarId_t logIdPmState;
static logVarId_t logIdVBat;
static logVarId_t logIdlighthouseEstBs0Rt;
static logVarId_t logIdlighthouseEstBs1Rt;
static paramVarId_t paramIdStabilizerController;
static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdLighthouseMethod;
static paramVarId_t paramIdCollisionAvoidanceEnable;
static paramVarId_t paramIdCollisionAvoidanceEllipsoidX;
static paramVarId_t paramIdCollisionAvoidanceEllipsoidY;
static paramVarId_t paramIdCollisionAvoidanceHorizon;
static paramVarId_t paramIdCollisionAvoidanceMaxVel;

static paramVarId_t paramIdCollisionAvoidanceBBoxMaxX;
static paramVarId_t paramIdCollisionAvoidanceBBoxMaxY;
static paramVarId_t paramIdCollisionAvoidanceBBoxMaxZ;
static paramVarId_t paramIdCollisionAvoidanceBBoxMinX;
static paramVarId_t paramIdCollisionAvoidanceBBoxMinY;
static paramVarId_t paramIdCollisionAvoidanceBBoxMinZ;

// NOTE: ICUAS24
static paramVarId_t zVelMax;
static paramVarId_t xVelMax;
static paramVarId_t yVelMax;
static paramVarId_t usdLogging;
static paramVarId_t usdcanLog;
static paramVarId_t posHold;

bool isLogging() {return paramGetInt(usdLogging) == 1; }
bool canLog() { return paramGetInt(usdcanLog) != 0; }
void setusdLogging(const uint8_t val) { return paramSetInt(usdLogging, val); }
// void setxVelMax(float val) { return paramSetFloat(xVelMax, val); }
// void setyVelMax(float val) { return paramSetFloat(yVelMax, val); }
// void setzVelMax(float val) { return paramSetFloat(zVelMax, val); }


// end ICUAS24

// getting  parameters
float getX() { return (float) logGetFloat(logIdStateEstimateX); }
float getY() { return (float) logGetFloat(logIdStateEstimateY); }
float getZ() { return (float) logGetFloat(logIdStateEstimateZ); }
float getVx() { return (float) logGetFloat(logIdStateEstimateVx); }
float getVy() { return (float) logGetFloat(logIdStateEstimateVy); }
float getVz() { return (float) logGetFloat(logIdStateEstimateVz); }
float getVelMagnitude() { return sqrtf(getVx()*getVx() + getVy()*getVy() + getVz()*getVz()); }
float getVarPX() { return logGetFloat(logIdKalmanVarPX); }
float getVarPY() { return logGetFloat(logIdKalmanVarPY); }
float getVarPZ() { return logGetFloat(logIdKalmanVarPZ); }
bool isBatLow() { return logGetInt(logIdPmState) == lowPower; }
float getVoltage() { return logGetFloat(logIdVBat); }
bool isCharging() { return logGetInt(logIdPmState) == charging; }
bool isLighthouseAvailable() { return logGetFloat(logIdlighthouseEstBs0Rt) >= 0.0f || logGetFloat(logIdlighthouseEstBs1Rt) >= 0.0f; }



void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }


void enableCollisionAvoidance() {
    if (!CollisionAvoidanceEnabled) {
        DEBUG_PRINT("Enabling Collision Avoidance\n");
        paramSetInt(paramIdCollisionAvoidanceEnable, 1);
        CollisionAvoidanceEnabled = true;
    }
}

void disableCollisionAvoidance() {
    if (paramGetInt(paramIdCollisionAvoidanceEnable) == 1) {
        DEBUG_PRINT("Disabling Collision Avoidance\n");
        paramSetInt(paramIdCollisionAvoidanceEnable, 0);
        CollisionAvoidanceEnabled = false;
    }
}

void initCollisionAvoidance(){
    paramSetFloat(paramIdCollisionAvoidanceEllipsoidX, COLLISION_AVOIDANCE_ELLIPSOID_XY_RADIUS);
    paramSetFloat(paramIdCollisionAvoidanceEllipsoidY, COLLISION_AVOIDANCE_ELLIPSOID_XY_RADIUS);
    paramSetFloat(paramIdCollisionAvoidanceHorizon, COLLISION_AVOIDANCE_HORIZON);
    paramSetFloat(paramIdCollisionAvoidanceMaxVel, COLLISION_AVOIDANCE_MAX_VELOCITY);

    paramSetFloat(paramIdCollisionAvoidanceBBoxMaxX, COLLISION_AVOIDANCE_BBOX_MAX_X);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMaxY, COLLISION_AVOIDANCE_BBOX_MAX_Y);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMaxZ, COLLISION_AVOIDANCE_BBOX_MAX_Z);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMinX, COLLISION_AVOIDANCE_BBOX_MIN_X);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMinY, COLLISION_AVOIDANCE_BBOX_MIN_Y);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMinZ, COLLISION_AVOIDANCE_BBOX_MIN_Z);

}

void initLogIds(){
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdStateEstimateVx = logGetVarId("stateEstimate", "vx");
    logIdStateEstimateVy = logGetVarId("stateEstimate", "vy");
    logIdStateEstimateVz = logGetVarId("stateEstimate", "vz");
}

void initParamLogInterface(){
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdKalmanVarPX = logGetVarId("kalman", "varPX");
    logIdKalmanVarPY = logGetVarId("kalman", "varPY");
    logIdKalmanVarPZ = logGetVarId("kalman", "varPZ");
    logIdPmState = logGetVarId("pm", "state");
    logIdVBat = logGetVarId("pm", "vbat");
    logIdlighthouseEstBs0Rt = logGetVarId("lighthouse", "estBs0Rt");
    logIdlighthouseEstBs1Rt = logGetVarId("lighthouse", "estBs1Rt");
    paramIdStabilizerController = paramGetVarId("stabilizer", "controller");
    paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
    paramIdLighthouseMethod = paramGetVarId("lighthouse", "method");
    paramIdCollisionAvoidanceEnable = paramGetVarId("colAv", "enable");
    paramIdCollisionAvoidanceEllipsoidX = paramGetVarId("colAv", "ellipsoidX");
    paramIdCollisionAvoidanceEllipsoidY = paramGetVarId("colAv", "ellipsoidY");
    paramIdCollisionAvoidanceHorizon = paramGetVarId("colAv", "horizon");
    paramIdCollisionAvoidanceMaxVel = paramGetVarId("colAv", "maxSpeed");
    paramIdCollisionAvoidanceBBoxMaxX = paramGetVarId("colAv", "bboxMaxX");
    paramIdCollisionAvoidanceBBoxMaxY = paramGetVarId("colAv", "bboxMaxY");
    paramIdCollisionAvoidanceBBoxMaxZ = paramGetVarId("colAv", "bboxMaxZ");
    paramIdCollisionAvoidanceBBoxMinX = paramGetVarId("colAv", "bboxMinX");
    paramIdCollisionAvoidanceBBoxMinY = paramGetVarId("colAv", "bboxMinY");
    paramIdCollisionAvoidanceBBoxMinZ = paramGetVarId("colAv", "bboxMinZ");

    // NOTE: ICUAS24
    xVelMax = paramGetVarId("posCtlPid", "xVelMax");
    yVelMax = paramGetVarId("posCtlPid", "yVelMax");
    zVelMax = paramGetVarId("posCtlPid", "zVelMax");
    usdLogging = paramGetVarId("usd", "logging");
    usdcanLog = paramGetVarId("usd", "canLog");
    posHold = paramGetVarId("flightmode", "poshold");
    // end ICUAS24

    initLogIds();
}

void initParamLogInterfaceSniffer() {
    usdLogging = paramGetVarId("usd", "logging");
    usdcanLog = paramGetVarId("usd", "canLog");
}

void reduceVelMax() {
    paramSetFloat(xVelMax, XY_VEL_MAX);
    paramSetFloat(yVelMax, XY_VEL_MAX);
    paramSetFloat(zVelMax, Z_VEL_MAX);
    paramSetInt(posHold, 1);
}