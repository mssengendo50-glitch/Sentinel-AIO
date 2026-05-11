#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <stdint.h>

/* * CLI Command Prototypes
 * These functions are called by the CLI parser when a matching 
 * string is entered in the terminal.
 */

/**
 * @brief Controls power rails (3v8, lora, lte, wifi, stm).
 * @param args String containing <rail_name> and <1|0>.
 */
void cmd_pwr(char *args);

/**
 * @brief Scans the specified I2C bus for active devices.
 * @param args String containing bus number (0 or 1).
 */
void cmd_i2cscan(char *args);

void cmd_hall(char *args);

void cmd_bq(char *args);

void cmd_gauge(char *args);

void cmd_sm(char *args);

void Run_Legacy_Monitors(char* processingBuffer);

#endif /* FUNCTIONS_H_ */