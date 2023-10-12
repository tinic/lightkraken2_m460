/*
MIT License

Copyright (c) 2023 Tinic Uro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef _WEBSERVER_H_
#define _WEBSERVER_H_

#include <stdint.h>

#include "fx_api.h"
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

#ifdef BOOTLOADER
    UINT postRequestUpload(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);
#endif  // #ifdef BOOTLOADER

    static UINT requestNotifyCallback(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);
    UINT requestNotify(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);

    static void APROMDiskDriver(FX_MEDIA *media_ptr);

    NX_HTTP_SERVER http_server {};
    FX_MEDIA ram_disk {};
    unsigned char media_memory[512];
};

#endif  // #ifndef _WEBSERVER_H_
