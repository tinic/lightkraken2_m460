#ifndef _WEBSERVER_H_
#define _WEBSERVER_H_

#include <stdint.h>

#include "tx_api.h"
#include "nx_api.h"
#include "nx_auto_ip.h"
#include "nxd_http_server.h"

#ifndef BOOTLOADER
#include "lwjson/lwjson.h"
#endif  // #ifndef BOOTLOADER

class WebServer {
public:
    static WebServer &instance();

    uint8_t *setup(uint8_t *pointer);
    bool start();

private:
    void init();
    bool initialized = false;

#ifndef BOOTLOADER
    static void jsonStreamSettingsCallback(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type);
    void jsonStreamSettings(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type);
    UINT postRequestJson(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr, lwjson_stream_parser_callback_fn callback);
#endif  // #ifndef BOOTLOADER

    static UINT requestNotifyCallback(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);
    UINT requestNotify(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);

    NX_HTTP_SERVER http_server {};
    FX_MEDIA ram_disk {};
    unsigned char media_memory[512];
};

#endif  // #ifndef _WEBSERVER_H_
