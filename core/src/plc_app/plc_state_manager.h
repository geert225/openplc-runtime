#ifndef PLC_STATE_MANAGER_H
#define PLC_STATE_MANAGER_H

#include "plcapp_manager.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PLC_STATE_INIT,
    PLC_STATE_RUNNING,
    PLC_STATE_STOPPED,
    PLC_STATE_ERROR,
    PLC_STATE_EMPTY
} PLCState;

/**
 * @brief Get the current PLC state.
 * @return PLCState The current PLC state
 */
PLCState plc_get_state(void);

/**
 * @brief Set the PLC state. In case of a state change, it will load or unload the PLC program as needed.
 * @param new_state The new PLC state to set
 * @return true if the state was changed, false if it was already in the desired state
 */
bool plc_set_state(PLCState new_state);

/**
 * @brief Cleanup the PLC state manager and unloads the plugin manager.
 * @return void
 */
void plc_state_manager_cleanup(void);

/**
 * @brief Force the PLC into ERROR state from any thread.
 *
 * This is intended for the watchdog thread to transition the PLC to ERROR
 * state without triggering program load/unload side effects (unlike plc_set_state).
 */
void plc_force_error_state(void);

/**
 * @brief Get the signal number that caused the last PLC crash.
 * @return The signal number (e.g. SIGFPE, SIGSEGV), or 0 if no crash occurred
 */
int plc_get_crash_signal(void);

#ifdef __cplusplus
}
#endif

#endif // PLC_STATE_MANAGER_H
