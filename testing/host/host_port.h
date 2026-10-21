/** @file host_port.h
 *  @brief Minimal micro T-Kernel API surface so shared application sources
 *         compile and run as native host tests (APP_HOST_TEST). Every stub
 *         records what it was asked to do; nothing blocks.
 */

#ifndef HOST_PORT_H
#define HOST_PORT_H

#include <stdint.h>

typedef int32_t  INT;
typedef uint32_t UINT;
typedef int32_t  ID;
typedef int32_t  ER;
typedef int32_t  SZ;
typedef int32_t  TMO;
typedef int32_t  PRI;
typedef uint32_t ATR;
typedef uint32_t RELTIM;
typedef uint8_t  UB;
typedef void (*FP)(void);

#define E_OK 0
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define E_QOVR (-43)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_HLNG 0x01U
#define TA_RNG3 0x300U
#define TA_TFIFO 0U
#define TA_WMUL 0x08U
#define TWF_ORW 0x01U
#define TWF_BITCLR 0x20U
#define TK_SUPPORT_SMP 0

typedef struct
{
    void * exinf;
    ATR    flgatr;
    UINT   iflgptn;
} T_CFLG;

typedef struct
{
    void * exinf;
    ATR    cycatr;
    FP     cychdr;
    RELTIM cyctim;
    RELTIM cycphs;
} T_CCYC;

typedef struct
{
    void * exinf;
    ATR    tskatr;
    FP     task;
    PRI    itskpri;
    SZ     stksz;
} T_CTSK;

ER  tk_set_flg (ID flgid, UINT setptn);
ER  tk_wup_tsk (ID tskid);
ER  tk_snd_mbf (ID mbfid, const void * msg, INT msgsz, TMO tmout);
ID  tk_get_tid (void);
ID  tk_cre_flg (const T_CFLG * pk);
ID  tk_cre_cyc (const T_CCYC * pk);
ER  tk_sta_cyc (ID cycid);
ID  tk_cre_tsk (const T_CTSK * pk);
ER  tk_sta_tsk (ID tskid, INT stacd);
INT tm_printf (const UB * fmt, ...);

/* host controls */
extern ID       host_current_tid;   /* what tk_get_tid returns */
extern uint32_t host_flg_set_count; /* tk_set_flg calls */
extern UINT     host_last_flg_ptn;
extern uint32_t host_mbf_send_count;
extern int      host_mbf_full; /* 1 = tk_snd_mbf returns E_TMOUT */
extern uint32_t host_mbf_last_record;
extern uint32_t host_irq_depth; /* > 0 while "interrupts disabled" */
extern uint32_t host_irq_max_depth;
extern void (*host_irq_hook)(void); /* called on the first restore to depth 0 */

#endif /* HOST_PORT_H */
