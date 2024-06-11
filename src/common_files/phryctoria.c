#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>

// #include "FreeRTOS.h"
// #include "semphr.h"
#include "debug.h"

#include "phryctoria.h"
#include "array_list.h"

// Global variables

uint8_t my_process;
List commA, pos_intervals, logs;
List commS, waiting_tokens, solver_events;
List satcuts;
List sending_tokens;

static int random_order[NUM_AGENTS]; 

static Event arr_comma[LIST_MAX_LENGTH];
static Interval arr_pos_intervals[LIST_MAX_LENGTH];
static Event arr_logs[LIST_MAX_EVENTS];

static CommS arr_comms[LIST_MAX_LENGTH];
static Token arr_waiting_tokens[LIST_MAX_LENGTH];
static Event arr_solver[LIST_MAX_EVENTS];

static Satcut arr_satcuts[LIST_MAX_LENGTH];

static Token arr_sending_tokens[LIST_MAX_LENGTH];


static void shuffle(int arr[], int n, int me) {
    int i;
    for(i = n-1; i >= 1; i--) {
        int j = rand() % (i+1);
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }
    // move my process to the front of the array
    for (int k=0; k<n; k++) {
        if (arr[k] == me) {
            int tmp = arr[0];
            arr[0] = me;
            arr[k] = tmp;
            break;
        }
    }
}

static const char *print_root_type(enum EType type) {
    switch (type) {
        case E_LEFT:
            return "locLeft";
        case E_RIGHT:
            return "locRight";
        case E_REMOTE:
            return "remRight";
        case E_VIRTUAL:
            return "virLeft";
        case E_NOTHING:
            return "Nothing";
        default:
            return "Error?";
    }
}

static const char *print_predicate(enum EPos pred) {
    switch (pred) {
        case E_NEG:
            return "false";
        case E_POS:
            return "true";
        case E_UNK:
            return "unknown";
        default:
            return "Error?";
    }
}

static const char *print_token_flag(enum TFlag flag) {
    switch (flag) {
        case T_AT:
            return "AT";
        case T_AFTER:
            return "AFTER";
        default:
            return "Error";
    }
}

void printPVC(Time pvc[]){
    DEBUG_PRINT("[");
    for (int i=0; i<NUM_AGENTS; i++)
        if (i == NUM_AGENTS-1)
            DEBUG_PRINT("%u", pvc[i]);
        else 
            DEBUG_PRINT("%u, ", pvc[i]);
    DEBUG_PRINT("]");
}

Time ms_to_s(uint32_t nowMS)
{
    if (nowMS >= 256000)
        return 0;
    else
        return (Time)((nowMS + 500) / 1000);
}

int compareEvents(void *a, void *b) {
    Event *aa = (Event *)a;
    Event *bb = (Event *)b;
    return (int)((int)aa->t - (int)bb->t);
}

int compareIntervals(void *a, void *b) {
    return (int)(((Interval *)a)->start - ((Interval *)b)->start);
}

void printEvent(void *e) {
    Event *ev = (Event *)e;
    DEBUG_PRINT("Event %u: t:%u, type:%s, ispos:%s, pvc:", ev->own_p, ev->t, print_root_type(ev->type), print_predicate(ev->ispos));
    printPVC(ev->pvc);
    DEBUG_PRINT("\n");
}

void printInterval(void *i) {
    DEBUG_PRINT("Interval: (start: %u, end: %u)\n", ((Interval *)i)->start, ((Interval *)i)->end);
}

void printToken(void *t) {
    Token *tk = (Token *)t;
    DEBUG_PRINT("Token %u: cnt:%u, tg_t:%u, flag:%s, tg_p:%u, type:%s, cut:", tk->own_p, tk->counter, tk->target_t, print_token_flag(tk->flag), tk->target_p, print_root_type(tk->type));

    printPVC(tk->cut);
    DEBUG_PRINT(", depend:");
    printPVC(tk->depend);
    DEBUG_PRINT(", ispos:[");
    for (int i=0; i<NUM_AGENTS; i++)
        DEBUG_PRINT("%s,", print_predicate(tk->ispos[i]));
    DEBUG_PRINT("], reset:[");
    for (int i=0; i<NUM_AGENTS; i++)
        DEBUG_PRINT("%d,", tk->reset[i]);
    DEBUG_PRINT("]\n");
}

void printCommS(void *s) {
    CommS *comms = (CommS *)s;
    DEBUG_PRINT("CommS: (type: %u)\n", comms->type);
    if (comms->type == S_TOKEN)
        printToken(&(comms->tk));
    else if (comms->type == S_EVENT)
        printEvent(&(comms->ev)); 
}

void printSatcut(void *c) {
    Satcut *sc = (Satcut *)c;
    printPVC(sc->pvc);
    DEBUG_PRINT(" type:%s\n", print_root_type(sc->type));
}

void list_print(List *list, const char *name) {
    if (name != NULL)
        DEBUG_PRINT("%s\n", name);
    _list_print(list);
}

void initPhryctoria(uint8_t address)
{
    // Address 0: sniffer, 1~N: agents
    // Phryctoria process: 0~(N-1): agents
    // process = address(i.e. my_id) - 1  
    my_process = address - 1;
    DEBUG_PRINT("Initialize Phryctoria, my_process: %u\n", my_process);

    for (int i=0; i<NUM_AGENTS; i++) {
        random_order[i] = i;
    }

    shuffle(random_order, NUM_AGENTS, (int)my_process);
    
    // for (int i=0; i<NUM_AGENTS; i++) {
    //     DEBUG_PRINT("%d, ", random_order[i]);
    // }

    // init Lists
    list_create(&commA, LIST_MAX_LENGTH, sizeof(Event), arr_comma, printEvent, NULL);
    list_create(&commS, LIST_MAX_LENGTH, sizeof(CommS), arr_comms, printCommS, NULL);
    list_create(&logs, LIST_MAX_EVENTS, sizeof(Event), arr_logs, printEvent, compareEvents);
    list_create(&pos_intervals, LIST_MAX_LENGTH, sizeof(Interval), arr_pos_intervals, printInterval, NULL);
    list_create(&waiting_tokens, LIST_MAX_LENGTH, sizeof(Token), arr_waiting_tokens, printToken, NULL);
    list_create(&sending_tokens, LIST_MAX_LENGTH, sizeof(Token), arr_sending_tokens, printToken, NULL);
    list_create(&solver_events, LIST_MAX_EVENTS, sizeof(Event), arr_solver, printEvent, compareEvents);
    list_create(&satcuts, LIST_MAX_LENGTH, sizeof(Satcut), arr_satcuts, printSatcut, NULL);

    // init my token and add to waiting token
    Token my_token;
    memset(&my_token, 0, sizeof(Token));
    my_token.own_p = my_process;
    my_token.target_p = my_process;
    my_token.flag = T_AFTER;

    list_append(&waiting_tokens, &my_token);
}

void initPhryctoriaSniffer() {
    my_process = 7;
    DEBUG_PRINT("Initialize Phryctoria for Sniffer\n");
    list_create(&satcuts, LIST_MAX_LENGTH, sizeof(Satcut), arr_satcuts, printSatcut, NULL);
}

void list_append_CommS(enum SType type, void *data) 
{
    static CommS newcomms;
    newcomms.type = type;
    if (type == S_EVENT)
        memcpy(&(newcomms.ev), (Event *)data, sizeof(Event));
    else if (type == S_TOKEN)
        memcpy(&(newcomms.tk), (Token *)data, sizeof(Token));

    list_append(&commS, &newcomms);
}

int locked(void) 
{
    Time *last_elem = arr_logs[logs.len-1].pvc;
    int i=0;
    for (; i<logs.len;i++) {
        bool isless = true;
        for (int j=0; j<NUM_AGENTS; j++) {
            isless &= (arr_logs[i].pvc[j] < last_elem[j]);
        }
        if (!isless)
            break;
    }
    return i;
}

enum EPos isPosInt(Time t)
{
    if (list_isempty(&pos_intervals))
        return E_UNK;
    if (t < arr_pos_intervals[0].start) 
        return E_NEG;
    for (int i=0; i<pos_intervals.len; i++) {
        if (t >= arr_pos_intervals[i].start && t <= arr_pos_intervals[i].end)
            return E_POS;
    }
    if (t >= arr_pos_intervals[pos_intervals.len-1].start && arr_pos_intervals[pos_intervals.len-1].end == 0)
        return E_UNK;
    if (t > arr_pos_intervals[pos_intervals.len-1].end && arr_pos_intervals[pos_intervals.len-1].end != 0)
        return E_UNK;
    return E_NEG;
}

void flushLog(const int count)
{
    // DEBUG_PRINT("FLUSH: %d events, current log:\n", count);
    // list_print(&logs, NULL);

    for (int i=0; i<count; i++) {
        Event e;
        list_take(&logs, &e);

        for (int j=0; j<NUM_AGENTS; j++) {
            e.pvc[j] = ((int)e.t - (int)SKEW) > e.pvc[j] ? (e.t-SKEW) : e.pvc[j];
        }
        // DEBUG_PRINT("Sending event [");
        // for (int j=0; j<NUM_AGENTS; j++) {
        //     DEBUG_PRINT("%u, ", e.pvc[j]);
        // }
        // DEBUG_PRINT("] (own_p:%u) to solver\n", e.own_p);
        // NOTE: E_UNK
        if (e.ispos == E_UNK) {
            e.ispos = isPosInt(e.t);
        }
        list_append_CommS(S_EVENT, &e);
    }
}

void updateIntervals(enum EType type, Time t)
{
    switch (type)
    {
        case E_LEFT:
        {
            Interval newi = {.start=t, .end=0};
            list_append(&pos_intervals, &newi);
            break;
        }
        case E_RIGHT:
        {
            assert(!list_isempty(&pos_intervals));
            arr_pos_intervals[pos_intervals.len-1].end = t;
            break;
        }
        default:
            break;
    }
}

// store max(PVC a, PVC b) to PVC a
static void maxPVC(Time a[], Time b[]) {
    for (int i=0; i<NUM_AGENTS; i++) {
        a[i] = a[i] >= b[i] ? a[i] : b[i];
    }
}

static void updateLogV(void) 
{
    Time V[NUM_AGENTS] = {0};
    for (int i=0; i<logs.len; i++) {
        maxPVC(arr_logs[i].pvc, V);
        maxPVC(V, arr_logs[i].pvc);
    }
}

// FIXME: possible bug source
void combine(Event *dst, Event *src)
{
    // DEBUG_PRINT("COMB: dst: ");
    // printEvent(dst);
    // DEBUG_PRINT("COMB: new: ");
    // printEvent(src);

    maxPVC(dst->pvc, src->pvc);
    if (src->type == E_LEFT || src->type == E_RIGHT) {
        dst->type = src->type;
    }
}

void updateLog(Event *event)
{
    for (int i=0; i<logs.len; i++) {
        if (event->t == arr_logs[i].t) {
            combine(&arr_logs[i], event);
            updateLogV();
            return;
        }
    }
    list_insert_sorted(&logs, event);
    updateLogV();
}

void foundRoot(Event *root)
{
    // DEBUG_PRINT("foundRoot\n");
    for (int i=0; i<logs.len; i++) {
        if (arr_logs[i].t > root->t)
            break;
        maxPVC(root->pvc, arr_logs[i].pvc);
    }

    if (root->type == E_RIGHT)
        broadcastRemoteRoot(root, 0);

    updateLog(root);
}

void remoteRoot(Event *remote)
{
    // DEBUG_PRINT("REMOTE: ");
    // printEvent(remote);
    updateLog(remote);
    int flush_count = locked();
    flushLog(flush_count);
}

void addEventToToken(Token *tok, Event ev) {
    // DEBUG_PRINT("ADD: ");
    // printEvent(&ev);

    // DEBUG_PRINT("ADD:BEFORE: ");
    // printToken(tok);

    // FIXME
    if (tok->own_p == my_process) {
        tok->type = ev.type;
        if (tok->type == E_LEFT) {
            for (int i=0; i<NUM_AGENTS; i++) {
                tok->reset[i] = true;
            }
        }
        else {
            for (int i=0; i<NUM_AGENTS; i++) {
                tok->reset[i] = false;
            }
        }
    }
    // if (tok->own_p == my_process) {
    //     tok->type = ev.type;
    //     if (tok->type != E_LEFT) {
    //         for (int i=0; i<NUM_AGENTS; i++)
    //             tok->reset[i] = false;
    //     }
    // }
        
    // if (ev.type == E_LEFT) {
    //     for (int i=0; i<NUM_AGENTS; i++)
    //         tok->reset[i] = true;
    // }

    tok->reset[my_process] = false;
    
    tok->cut[my_process] = ev.t;
    tok->ispos[my_process] = ev.ispos;
    maxPVC(tok->depend, ev.pvc);

    // DEBUG_PRINT("ADD:AFTER : ");
    // printToken(tok);
}

bool isConsistentCut(Token tok, uint8_t *incon_id) {
    int i = 0;
    for (int j=0; j<NUM_AGENTS; j++) {
        i = random_order[j];
        if (tok.cut[i] == 0 || tok.depend[i] > tok.cut[i]) {
            *incon_id = i;
            return false;
        }
    }
    return true;
}

enum EPos isSatCut(Token tok, uint8_t *forbid_id) {
    int i = 0;
    for (int j=0; j<NUM_AGENTS; j++) {
        i = random_order[j];
        if (tok.ispos[i] == E_UNK) {
            *forbid_id = i;
            return E_UNK;
        }
        if (tok.ispos[i] == E_NEG) {
            *forbid_id = i;
            return E_NEG;
        }
    }
    return E_POS;
}

void sendToken(Token tok) {
    tok.send_p = my_process;
    tok.counter++;
    // DEBUG_PRINT("SEND: From %u To %u, : ", my_process, tok.target_p);
    // printToken(&tok);
    // DEBUG_PRINT("\n");

    // list_append(&sending_tokens, &tok);
    if (tok.target_p == my_process) {
        // send to myself
        list_append_CommS(S_TOKEN, &tok);
    }
    else {
        list_append(&sending_tokens, &tok);
        // broadcastToken(&tok);
    }
}

void evaluateToken(Token *tok) {
    // DEBUG_PRINT("EVAL: ");
    // printToken(tok);
    
    static uint8_t forbid_id;
    tok->eval = isSatCut(*tok, &forbid_id);

    if (tok->eval == E_POS) {
        tok->target_p = tok->own_p;
    }
    else if (tok->eval == E_NEG) {
        tok->target_p = forbid_id;
        tok->flag = T_AFTER;
        // optimization
        // if (forbid_id != tok->own_p) {
        //     tok->target_t = tok->depend[forbid_id] - 1;
        // }
        // else {
        //     tok->target_t = tok->cut[forbid_id];
        // }
        tok->target_t = tok->cut[forbid_id];
    }
    else {
        // E_UNK
        tok->target_p = forbid_id;

        if (tok->reset[forbid_id]) {
            tok->flag = T_AT;
            tok->target_t = tok->depend[forbid_id];
        }
    }
    sendToken(*tok);
}

void processToken(Token *tok) {
    // NOTE: handle E_UNK
    if (tok->ispos[my_process] == E_UNK) {
        tok->target_p = my_process;
        sendToken(*tok);
        return;
    }
    
    static uint8_t incon_id;

    if (isConsistentCut(*tok, &incon_id)) {
        // consistent cut
        // evaluateToken
        // DEBUG_PRINT("PROC: Consistent cut!\n");
        evaluateToken(tok);
    }
    else {
        // inconsistent cut
        // DEBUG_PRINT("PROC: Inconsistent cut, target process: %u\n", incon_id);
        tok->target_p = incon_id;
        tok->eval = E_NEG;
        if (tok->cut[incon_id] == 0 || tok->reset[incon_id]) {
            tok->flag = T_AT;
            tok->target_t = tok->depend[incon_id];
        }
        else {
            tok->flag = T_AFTER;
            tok->target_t = tok->cut[incon_id];
            // optimization
            // if (incon_id != tok->own_p) {
            //     tok->target_t = tok->depend[incon_id];
            // }
            // else {
            //     tok->target_t = tok->cut[incon_id];
            // }
        }
        sendToken(*tok);
    }
}

void addVirtEvent(Time t, Event *ev) {
    // prevent duplicate virtual event
    for (int i=0; i<solver_events.len; i++) {
        if (arr_solver[i].t == t) {
            *ev = arr_solver[i];
            return;
        }
    }
    
    ev->own_p = my_process;
    ev->ispos = isPosInt(t);
    ev->t = t;
    ev->type = E_VIRTUAL;
    for (int i=0; i<NUM_AGENTS; i++) {
        ev->pvc[i] = ((int)t - (int)SKEW) > 0 ? (t - SKEW) : 0;
    }
    ev->pvc[my_process] = t;

    // list_print(&solver_events, "ADDV: BEFORE, solver_events");
    list_insert_sorted(&solver_events, ev);
    // list_print(&solver_events, "ADDV: AFTER , solver_events");
}

void receiveEvent(Event ev) {
    // DEBUG_PRINT("RECVE: ");
    // printEvent(&ev);

    // list_print(&solver_events, "RECVE: BEFORE, solver_events");
    list_insert_sorted(&solver_events, &ev);
    // list_print(&solver_events, "RECVE: AFTER , solver_events");

    // NOTE: E_UNK
    for (int i=0; i<solver_events.len; i++) {
        if (arr_solver[i].ispos == E_UNK) {
            arr_solver[i].ispos = isPosInt(arr_solver[i].t);
        }
    }
    // NOTE: E_UNK
    for (int i=0; i<waiting_tokens.len; i++) {
        if (arr_waiting_tokens[i].ispos[my_process] == E_UNK) {
            enum EPos newpos = isPosInt(arr_waiting_tokens[i].cut[my_process]);
            if (newpos != E_UNK) {
                arr_waiting_tokens[i].ispos[my_process] = newpos;
                static Token tk;
                memcpy(&tk, &arr_waiting_tokens[i], sizeof(Token));
                list_remove(&waiting_tokens, i);
                processToken(&tk);
            }
        }
    }

    int loop = waiting_tokens.len;
    // list_print(&waiting_tokens, "RECVE: waiting tokens");
    for (int i=0; i<loop; i++) {
        if (!list_isempty(&waiting_tokens)) {
            static Token tk;
            list_take(&waiting_tokens, &tk);
            // NOTE: E_UNK
            if (tk.ispos[my_process] == E_UNK) {
                list_append(&waiting_tokens, &tk);
                continue;
            }
            if (tk.flag == T_AT && tk.target_t == ev.t) {
                addEventToToken(&tk, ev);
                processToken(&tk);
            }
            else if (tk.flag == T_AFTER && tk.target_t < ev.t) {
                addEventToToken(&tk, ev);
                processToken(&tk);
            }
            else if (tk.flag == T_AT && tk.target_t < ev.t) {
                // addVirtEvent
                Event virev = {0};
                addVirtEvent(tk.target_t, &virev);
                addEventToToken(&tk, virev);
                processToken(&tk);
            }
            else if (tk.flag == T_AT) {
                list_append(&waiting_tokens, &tk);
                continue;
            }
            else {
                // DEBUG_PRINT("Something in the token's target isn't looking right: \n");
                list_append(&waiting_tokens, &tk);
            }
        }
    }
}

void receiveToken(Token tk) {
    // DEBUG_PRINT("RECVT: ");
    // printToken(&tk);

    // NOTE: E_UNK
    if (tk.ispos[my_process] == E_UNK) {
        list_append(&waiting_tokens, &tk);
        return;
    }

    if (tk.eval == E_POS && tk.own_p == my_process) {
        Satcut sc = {0};
        sc.send_p = my_process;
        sc.type = tk.type;
        for (int i=0; i<NUM_AGENTS; i++) {
            sc.pvc[i] = tk.cut[i];
        }
        list_append(&satcuts, &sc);
        // list_print(&satcuts, "SATCUT");
        broadcastSatCut(&sc);
        // reset this token
        tk.eval = E_NEG;
        tk.target_p = my_process;
        tk.flag = T_AFTER;
        tk.target_t = tk.cut[my_process];
        tk.type = E_NOTHING;
        // for (int i=0; i<NUM_AGENTS; i++) {
        //     tk.reset[i] = false;
        //     tk.ispos[i] = E_UNK;
        // }
        // FIXME the bug?
        // list_append(&waiting_tokens, &tk);
        // return;
        // DEBUG_PRINT("RECVT: AFTER SAT: ");
        // printToken(&tk);
    }

    // searchsortedfirst
    int index = 0;
    for (; index < solver_events.len; index++) {
        if (arr_solver[index].t >= tk.target_t)
            break;
    }
    
    // list_print(&solver_events, "RECVT: Current Solver events");

    if (tk.target_t > arr_solver[solver_events.len-1].t) {
        list_append(&waiting_tokens, &tk);
        return;
        // // NOTE: the return above makes below meaningless?
        // if (tk->flag == T_AT)
        //     return;
        // else 
        //     DEBUG_PRINT("Why didn't this event get added to events?\n");
    }

    if (tk.flag == T_AFTER) {
        if (tk.target_t == arr_solver[solver_events.len-1].t){
            list_append(&waiting_tokens, &tk);
            return;
        }
        else if (tk.target_t == arr_solver[index].t) {
            addEventToToken(&tk, arr_solver[index+1]);
            processToken(&tk);
        }
        else {
            addEventToToken(&tk, arr_solver[index]);
            processToken(&tk);
        }
    }
    else {  // tk.flag == T_AT
        if (tk.target_t == arr_solver[index].t) {
            addEventToToken(&tk, arr_solver[index]);
            processToken(&tk);
        }
        else {
            Event virev = {0};
            addVirtEvent(tk.target_t, &virev);
            addEventToToken(&tk, virev);
            processToken(&tk);
        }
    }
}

