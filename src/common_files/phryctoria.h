#ifndef __PHRYCTORIA_H__
#define __PHRYCTORIA_H__

/**
 * phryctoria.h
 */

#include <stdint.h>
#include <stdbool.h>

#include "array_list.h"

// Address 0: sniffer, 1~N: agents
// Phryctoria process: 0~(N-1): agents
// process = address(i.e. my_id) - 1  
#define NUM_AGENTS 7 // Number of flying agents.
#define MAX_AGENTS 7
#define SKEW 1 // Clock skew

#define LIST_MAX_LENGTH     50
#define LIST_MAX_EVENTS     200

// NOTE: phryctoria, predicate condition
#define Z_PREDICATE 0.7f
#define Z_CHANGE 0.3f
#define PRED(X) ((X) < (Z_PREDICATE) ? 1 : 0)

#define MISSION_ON_MAX     14000
#define MISSION_ON_MIN     2000
#define MISSION_OFF_MAX    5000
#define MISSION_OFF_MIN    2000
#define MISSION_DURATION   5000

#define SEND_DELAY_MAX          70
#define SEND_DELAY_MIN          10
#define SEND_ROOT_REPEAT        2
#define SEND_TOKEN_REPEAT       3
#define LOG_OFF_DELAY           6000

#define RESEND_TOKEN_TIMEOUT    5000    
#define RESEND_TOKEN_TIMES      1

#define RESEND_EVENT_TIMEOUT    1000    
#define RESEND_EVENT_TIMES      1

// Message type
enum {
    ROOT,
    TOKEN,
    DATA,   // groundtruth recording for sniffer
    SATCUT
} __attribute__((packed));

// enums for an event
enum EType {
    E_NOTHING = 0,
    E_LEFT,
    E_RIGHT,
    E_REMOTE,
    E_VIRTUAL,
} __attribute__((packed));

enum EPos {
    E_NEG = 0,
    E_POS,
    E_UNK
} __attribute__((packed));

enum TFlag {
    T_AT = 0,
    T_AFTER
} __attribute__((packed));

enum SType {
    S_TOKEN = 0,
    S_EVENT
} __attribute__((packed));

// for logging
enum {
    FOUNDROOT = 0,
    REMOTEROOT
};

enum {
    RECVEVENT = 0,
    RECVTOKEN
};

enum {
    TOKEN_SEND = 0,
    TOKEN_RECV
};
// for logging


// Default unit for time in Crazyflie is milisecond.
// To reduce message size, it will be converted to second.
// Milisecond is still used for clock synchronization.
// 0~255 sec
typedef uint8_t Time;

// NOTE: maximum data size for message - 59 bytes
// 1 byte for message category
// Event Info
typedef struct
{
    uint8_t own_p;      // who created this event at first? for remoteroot my_process != own_p
    enum EType type;
    enum EPos ispos;
    Time t;
    Time pvc[NUM_AGENTS];        // Physical Vector Clock
} Event;
// sizeof(Event) = 4 + N

// Token
typedef struct {
    uint8_t own_p;            // The agent who owns this token
    enum EType type;
    bool reset[NUM_AGENTS];
    Time cut[NUM_AGENTS];
    Time depend[NUM_AGENTS];
    enum EPos ispos[NUM_AGENTS];
    uint8_t target_p;
    enum TFlag flag;
    Time target_t;
    enum EPos eval;
    uint16_t counter;   // for debugging
    uint8_t send_p;
} Token;
// sizeof(Token) = 9 + 4 * N;
// N=7 => 37

typedef struct {
    enum SType type;    // event or token
    union {
        Event ev;
        Token tk;
    };
} CommS;

typedef struct {
    uint8_t send_p;
    enum EType type;
    Time pvc[NUM_AGENTS];
} Satcut;

typedef struct {
    Time start;
    Time end;
} Interval;

extern uint8_t my_process;
extern List commA, pos_intervals, logs;
extern List commS, waiting_tokens, solver_events;
extern List satcuts;
extern List sending_tokens;


void broadcastRemoteRoot(Event *rightroot, int repeat);

// void broadcastToken(Token *token);

void broadcastSatCut(Satcut *s);

void initPhryctoria(uint8_t address);

void initPhryctoriaSniffer();

void updateIntervals(enum EType type, Time t);

Time ms_to_s(uint32_t nowMS);

void foundRoot(Event *root);

void remoteRoot(Event *remote);

void list_append_CommS(enum SType type, void *data);

enum EPos isPosInt(Time t);

void receiveEvent(Event ev);

void receiveToken(Token tk);

void printEvent(void *e);

void printInterval(void *i);

void printSatcut(void *c);

void printToken(void *t);

void printComms(void *s);

void list_print(List *list, const char *name);


#endif // 