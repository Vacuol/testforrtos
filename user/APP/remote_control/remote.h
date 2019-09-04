#ifndef REMOTE_H
#define REMOTE_H


/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#include "main.h"

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[4];
                char sleft;
								char sright;
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
								uint16_t v2;
        } key;

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;
extern uint8_t teledata_rx[18];

void SBUS_TO_RC(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

#endif
