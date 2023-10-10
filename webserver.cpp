#include "webserver.h"
#include "network.h"

WebServer &WebServer::instance() {
    static WebServer webserver;
    if (!webserver.initialized) {
        webserver.initialized = true;
        webserver.init();
    }
    return webserver;
}

void WebServer::init() {
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

UINT request_notify(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr)
{
    if (strcmp(resource, "/settings") == 0) {
        switch(request_type) {
            case NX_HTTP_SERVER_GET_REQUEST: {
                nx_http_server_callback_response_send_extended(server_ptr, "HTTP/1.0 404 ", 12, "NetX HTTP Server unable to find file: ", 38, resource, 9);
                return(NX_HTTP_CALLBACK_COMPLETED);
            }
            case NX_HTTP_SERVER_POST_REQUEST:
            case NX_HTTP_SERVER_PUT_REQUEST: {
                nx_http_server_callback_response_send_extended(server_ptr, "HTTP/1.0 404 ", 12, "NetX HTTP Server unable to find file: ", 38, resource, 9);
                return(NX_HTTP_CALLBACK_COMPLETED);
            }
        }
    }
    return(NX_SUCCESS);
}

#pragma GCC diagnostic pop

uint8_t *WebServer::setup(uint8_t *pointer) {
    UINT status;

    const size_t http_server_stack_size = 2048;

    fx_system_initialize();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

    status = nx_http_server_create(&http_server, "WebServer", Network::instance().ip(), &ram_disk, pointer, http_server_stack_size, Network::instance().pool(), NX_NULL, request_notify);
    pointer = pointer + http_server_stack_size;
    if (status) {
        goto fail;
    }

    static NX_HTTP_SERVER_MIME_MAP map[] = {
        {"js",     "text/javascript"},
        {"css",    "text/css"},
        {"json",   "application/json"},
        {"svg",    "image/svg+xml"}
    };

#pragma GCC diagnostic pop

    nx_http_server_mime_maps_additional_set(&http_server, map, 4);

    return pointer;

fail:
    while(1) {}
}

__attribute__((section(".text#")))
#include "fs.h"

extern "C" void _fx_ram_driver(FX_MEDIA *media_ptr);

bool WebServer::start() {
    UINT status;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

    status = fx_media_open(&ram_disk, "RAM Disk", _fx_ram_driver, fs_data, media_memory, sizeof(media_memory));
    if (status) {
        return false;
    }

#pragma GCC diagnostic pop

    status = nx_http_server_start(&http_server);
    if (status) {
        return false;
    }

    return true;
}
