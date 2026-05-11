#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

/* =========================
   Configuration Macros
   ========================= */

#define MAX_COMMANDS     10
#define MAX_INPUT_LEN    128
#define MAX_TOKENS       10
#define MAX_BUFFER_SIZE  64
#define MAX_ARGS_LEN     64

/* =========================
   Global UART State
   ========================= */

extern char UART_Buffer[MAX_BUFFER_SIZE];
extern volatile bool data_received;
extern volatile uint8_t char_index;

/* =========================
   UART Core Functions
   ========================= */

void uart_init(void);
void UARTReceive(void);
void get_UART_buffer(char* output_buffer);

int uart_printf(const char *fmt, ...);

void printf_user(char* string, uint8_t string_length, char end_char);
void printToUART(char* string, char end_char);

/* =========================
   CLI Types
   ========================= */

typedef void (*CommandFunc)(char *args);

typedef struct {
    const char *name;
    CommandFunc func;
    const char *help_desc;
} CLI_Command;

/* =========================
   CLI Interface Functions
   ========================= */

void CLI_RegisterCommand(const char *name, CommandFunc func, const char* help_desc);
int  CLI_Tokenize(char *input, char *tokens[], int maxTokens);
void CLI_ProcessInput(char *input);

/* =========================
   Command Handlers
   (Added these so main.c can register them)
   ========================= */

void cmd_help(char *args);
void cmd_pwr(char *args);
void cmd_log(char *args);  // Added if you still have the log function
void cmd_chg(char *args);  // Added if you still have the charge function

#endif /* UART_H */