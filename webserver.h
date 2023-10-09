#ifndef _WEBSERVER_H_
#define _WEBSERVER_H_

#include <stdint.h>

#include "tx_api.h"
#include "nx_api.h"
#include "nx_auto_ip.h"
#include "nxd_http_server.h"

class WebServer {
public:
    static WebServer &instance();

    uint8_t *setup(uint8_t *pointer);
    bool start();

private:
    void init();
    bool initialized = false;

    NX_HTTP_SERVER http_server {};
    FX_MEDIA ram_disk {};
    unsigned char media_memory[512];
};

#endif  // #ifndef _WEBSERVER_H_
