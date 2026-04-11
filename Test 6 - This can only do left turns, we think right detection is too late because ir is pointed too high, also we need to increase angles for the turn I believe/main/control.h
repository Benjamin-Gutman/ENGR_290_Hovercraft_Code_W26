#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
//Calling in main loop
//Applies the control outputs based on the current craft state
void apply_control(void);
#ifdef __cplusplus
}
#endif
#endif