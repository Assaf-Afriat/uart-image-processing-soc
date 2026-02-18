// Shared types for UART modules

package uart_types;

    typedef enum logic [2:0] {
        IDLE,
        START_BIT,
        DATA_BITS,
        STOP_BIT
    } uart_state_t;

endpackage

