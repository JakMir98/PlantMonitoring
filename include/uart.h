#ifndef UART_H
#define UART_H

void uart_init();

static void echo_task(void *arg);

int sendData(const char* logName, const char* data);
static void tx_task(void *arg);
static void rx_task(void *arg);

#endif
