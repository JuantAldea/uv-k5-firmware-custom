#ifndef APP_TERMINAL_H
#define APP_TERMINAL_H

bool TERMINAL_Init();
void TERMINAL_Exit();
void TERMINAL_UpdateScreen();
void TERMINAL_putchar(char c);
void TERMINAL_Demo();

#endif