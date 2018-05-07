/******************************************************************************
 *  app.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Task main_task
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "ev3api.h"

/*
 *  Šeƒ^ƒXƒN‚Ì—Dæ“x‚Ì’è‹`
 */
#define MAIN_PRIORITY    TMIN_APP_TPRI + 1  /* ƒƒCƒ“ƒ^ƒXƒN‚Ì—Dæ“x */
#define TRACER_PRIORITY  TMIN_APP_TPRI + 2
#define MOD_PRIORITY	 TMIN_APP_TPRI + 3 


/*
 *  ƒ^[ƒQƒbƒg‚ÉˆË‘¶‚·‚é‰Â”\«‚Ì‚ ‚é’è”‚Ì’è‹`
 */
#ifndef STACK_SIZE
#define STACK_SIZE      4096        /* ƒ^ƒXƒN‚ÌƒXƒ^ƒbƒNƒTƒCƒY */
#endif /* STACK_SIZE */

/*
 *  ŠÖ”‚Ìƒvƒƒgƒ^ƒCƒvéŒ¾
 */
#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void tracer_task(intptr_t exinf);
extern void bt_task(intptr_t exinf);
extern void ev3_cyc_tracer(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
