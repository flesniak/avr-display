#pragma once

typedef struct {
    unsigned short channel;
    bool dmx_sending:1, dmx_receiving:1;
    unsigned char override_count:2;
    unsigned short override_value[3];
} config_t;

// typedef struct {
//     unsigned short channel;
//     bool dmx_sending:1, dmx_receiving:1;
//     unsigned char override_count:2;
//     unsigned short override_value[3];
// } config_t_test;

// char (*__kaboom)[sizeof( config_t_test )] = 1;
