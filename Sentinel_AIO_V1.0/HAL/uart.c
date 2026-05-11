/**
 * @brief function to print formatted strings over UART channel, takes variable list of arguments
 * 
 * @param fmt  format string for data to be printed to UART
 * @param ...  args to be put in the format string
 * @return int  number of characters output. Can be used to ascertain function has output correctly.
 *              Max output is 256 characters at a time (buffer limit).
 */

 #include <string.h>
 #include <stdio.h>
 #include <stdarg.h>
 #include <stdlib.h>
 #include "ti_msp_dl_config.h"
 #include "uart.h"
int uart_printf(const char *fmt, ...);

// import string.h and stdrarg.h
#define MAX_COMMANDS 10
#define MAX_INPUT_LEN 128
#define MAX_TOKENS 10
//UART


#define MAX_BUFFER_SIZE 64 // 17 bytes plus null character of string

char UART_Buffer[MAX_BUFFER_SIZE];
volatile bool data_received = false;
volatile uint8_t char_index = 0;


__STATIC_INLINE void invokeBSLAsm(void)
{
    /* Erase SRAM completely before jumping to BSL */
    __asm(
#if defined(__GNUC__)
        ".syntax unified\n" /* Load SRAMFLASH register*/
#endif
        "ldr     r4, = 0x41C40018\n" /* Load SRAMFLASH register*/
        "ldr     r4, [r4]\n"
        "ldr     r1, = 0x03FF0000\n" /* SRAMFLASH.SRAM_SZ mask */
        "ands    r4, r1\n"           /* Get SRAMFLASH.SRAM_SZ */
        "lsrs    r4, r4, #6\n"       /* SRAMFLASH.SRAM_SZ to kB */
        "ldr     r1, = 0x20300000\n" /* Start of ECC-code */
        "adds    r2, r4, r1\n"       /* End of ECC-code */
        "movs    r3, #0\n"
        "init_ecc_loop: \n" /* Loop to clear ECC-code */
        "str     r3, [r1]\n"
        "adds    r1, r1, #4\n"
        "cmp     r1, r2\n"
        "blo     init_ecc_loop\n"
        "ldr     r1, = 0x20200000\n" /* Start of NON-ECC-data */
        "adds    r2, r4, r1\n"       /* End of NON-ECC-data */
        "movs    r3, #0\n"
        "init_data_loop:\n" /* Loop to clear ECC-data */
        "str     r3, [r1]\n"
        "adds    r1, r1, #4\n"
        "cmp     r1, r2\n"
        "blo     init_data_loop\n"
        /* Force a reset calling BSL after clearing SRAM */
        "str     %[resetLvlVal], [%[resetLvlAddr], #0x00]\n"
        "str     %[resetCmdVal], [%[resetCmdAddr], #0x00]"
        : /* No outputs */
        : [ resetLvlAddr ] "r"(&SYSCTL->SOCLOCK.RESETLEVEL),
        [ resetLvlVal ] "r"(DL_SYSCTL_RESET_BOOTLOADER_ENTRY),
        [ resetCmdAddr ] "r"(&SYSCTL->SOCLOCK.RESETCMD),
        [ resetCmdVal ] "r"(
            SYSCTL_RESETCMD_KEY_VALUE | SYSCTL_RESETCMD_GO_TRUE)
        : "r1", "r2", "r3", "r4");
}


//initialize uart
void uart_init(void){
    data_received = false;
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void printf_user(char* string, uint8_t string_length, char end_char){
    for(uint8_t i = 0; i<string_length; i++){
        if(string[i] == end_char){
            putchar(string[i]);
            break;
        }
        putchar(string[i]);
    }
}

void printToUART(char* string, char end_char){
    uint32_t i = 0;
    while(1){
        DL_UART_Main_transmitDataBlocking(UART_0_INST, string[i]);
        i++;
        if(string[i] == end_char){
            break;
        }
    }
}

//receive bytes from uart upto 8 bytes or when carriage return/new line character is encountered. whichever happens first
//store in the UART buffer and set buffer to have new data
void UARTReceive(){
    if(data_received == true){
        DL_UART_receiveData(UART_0_INST);
        return;
    }
    // printf("Character: %d\n", char_index);
    char c = DL_UART_receiveData(UART_0_INST);

    if (c == 0x22) {
        invokeBSLAsm();
        return;
    }
    
    if(c == '\r' && char_index == 0) return;

    UART_Buffer[char_index] = c;
    char_index = char_index + 1;
    // printf("%s\n", UART_Buffer);
    if(char_index > MAX_BUFFER_SIZE - 2 || c == '\n' || c == '~'){
        if(c == '\n'){
            // printf("NewLine\n");
        }

        if(c == '\r'){
            // printf("CarriageReturn\n");
        }
        UART_Buffer[char_index] = '\0';
        data_received = true;
        char_index = 0;
        // printf("Received: %s\n", UART_Buffer);
    }
};

//get uart buffer contents
void get_UART_buffer(char* output_buffer){
    
    strcpy(output_buffer, UART_Buffer);

    // printf("UART_Buffer: %s\n", UART_Buffer);
    memset(UART_Buffer, 0, MAX_BUFFER_SIZE);
    // printToUART(output_buffer, '\0');
    
    data_received = false;
}


/**
 * @brief CLI command function type
 * 
 */
typedef void (*CommandFunc)(char *args);



/**
 * @brief function to print formatted strings over UART channel, takes variable list of arguments
 * 
 * @param fmt  format string for data to be printed to UART
 * @param ...  args to be put in the format string
 * @return int  number of characters output. Can be used to ascertain function has output correctly.
 *              Max output is 256 characters at a time (buffer limit).
 */
int uart_printf(const char *fmt, ...){
    char buffer[768];           // Adjust buffer size as needed
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0) {
        return len;   // formatting error
    }

    if (len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;  // actual transmitted length
    }

    printToUART(buffer, '\0');


    return len;
}


//CLI

#define out uart_printf //change if need different output channel
#define MAX_ARGS_LEN 64


static CLI_Command commandList[MAX_COMMANDS];
static int commandCount = 0;

/**
 * @brief function to register CLI commands
 * 
 * @param name - string of command
 * @param func - handler function name for the command. This where all its processing happens
 * @param help_desc - string describing function use / purpose
 */
void CLI_RegisterCommand(const char *name, CommandFunc func, const char* help_desc) {
    if (commandCount < MAX_COMMANDS) {
        commandList[commandCount].name = name;
        commandList[commandCount].func = func;
        commandList[commandCount].help_desc = help_desc;
        commandCount++;
    } else {
        out("Error: Command limit reached (%d)\n", MAX_COMMANDS);
    }
}

/**
 * @brief Function for extracting parameters passed aside from command to CLI
 * 
 * @param input - string input from buffer
 * @param tokens - output array to hold received values
 * @param maxTokens - maximum number of tokens to be extracted
 * @return int - number of args extracted from the input
 */
int CLI_Tokenize(char *input, char *tokens[], int maxTokens) {
    int count = 0;
    char *token = strtok(input, " ");
    while (token && count < maxTokens) {
        tokens[count++] = token;
        token = strtok(NULL, " ");
    }
    return count;
}

/**
 * @brief Processes passed input in regards to the registered commands
 * 
 * @param input - string input
 */
void CLI_ProcessInput(char *input) {
    char *tokens[MAX_TOKENS];
    int tokenCount;
    char args[MAX_ARGS_LEN];

    // Remove newline if present
    input[strcspn(input, "\r\n")] = '\0';

    char *arg_start = strchr(input, ' ');
    if (arg_start) {
        while (*arg_start == ' ') arg_start++;
        strncpy(args, arg_start, sizeof(args) - 1);
        args[sizeof(args) - 1] = '\0';
    } else {
        args[0] = '\0';
    }

    tokenCount = CLI_Tokenize(input, tokens, MAX_TOKENS);
    if (tokenCount == 0) return;

    for (int i = 0; i < commandCount; i++) {
        if (strcmp(tokens[0], commandList[i].name) == 0) {
            commandList[i].func(args);
            return;
        }
    }

    out("Unknown command: %s\n", tokens[0]);
}

void cmd_help(char* args){
    uart_printf("Available commands:\n\n");
    for(int i=0; i < commandCount; i++){
        uart_printf("\t%s: %s\n", commandList[i].name, commandList[i].help_desc);
    }
}

