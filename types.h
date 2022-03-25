#pragma once

typedef struct {
    unsigned short channel;
    bool say_hello:1, dmx_sending:1, dmx_receiving:1;
    unsigned char override_count:2;
    unsigned short override_value[3];
} config_t;

// char (*__kaboom)[sizeof( config_t )] = 1;
