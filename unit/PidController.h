/******************************************************************************
 *  PidController.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/06/12
 *  Implementation of the Class BalancingWalker
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_UNIT_PIDCONTROLLER_H_
#define EV3_UNIT_PIDCONTROLLER_H_

#include "ev3api.h"


class PidController
{
public:
    PidController();
    virtual ~PidController();

    /**
     * PIDêßå‰Ç≈TurnílÇéZèoÇ∑ÇÈ
     */
    float calControlledVariable(int8_t deviation);
    void setPidFactor(float kp, float ki, float kd);
    void setEdge(int edge);

private:
    int32_t mDeviation;
    int32_t mIntegral;

    float KPID_P_FACTOR;
    float KPID_I_FACTOR;
    float KPID_D_FACTOR;

    int mEdge;
};

#endif  // EV3_UNIT_PIDCONTROLLER_H_
