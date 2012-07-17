#define END			192
#define ESC			219
#define ESC_END	220
#define ESC_ESC	221
#define STARTx		193

//extern void slip_tx(uint8_t *buf, int size, uint8_t *EndOfBuff, uint32_t MaxSizeBuff);
extern uint8_t slip_tx (uint8_t * buf, int size, uint8_t * EndOfBuff,
                        uint32_t MaxSizeBuff, uint8_t * buf_out);
