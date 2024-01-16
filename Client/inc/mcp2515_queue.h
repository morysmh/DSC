#ifndef	MCP2515_QUEUE_H__
#define	MCP2515_QUEUE_H__
#include <stdint.h>
#include "define.h"
#include "mcp2515.h"
#ifdef __cplusplus
extern "C"
{
#endif

void mcp2515_buffer_run();

uint8_t mcp2515_write_queue(tCAN *i_data,int8_t i_priority);

uint16_t rb_add(volatile uint16_t *i_head,int8_t i_add);
void copy_tCAN(tCAN *o_dest,tCAN *i_source);
#ifdef __cplusplus
}
#endif

#endif