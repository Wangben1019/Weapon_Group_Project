#ifndef _RECEIVE_JOY_NODE_H_
#define _RECEIVE_JOY_NODE_H_

#include "qbytearray.h"
#include "VCOMCOMM.h"
#include "memory"
#include <memory>

typedef struct{
    float Forward_Back_Remote;
    float Left_Right_Remote;
    float Rotate_Remote;
    float Stop_Remote;
} Expect_Speed_Typedef;

#endif
