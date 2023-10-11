#include "webserver.h"
#include "network.h"
#include "lwjson/lwjson.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

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

#ifndef BOOTLOADER
void WebServer::jsonStreamSettingsCallback(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type) {
    WebServer::instance().jsonStreamSettings(jsp, type);
}

void WebServer::jsonStreamSettings(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type) {
    if (jsp == NULL) {
        return;
    }
    if (jsp->stack_pos != 2) {
        return;
    }

    const char *key_name = jsp->stack[jsp->stack_pos-1].meta.name;
    const char *data_buf = jsp->data.str.buff;
    switch(type) {
        case LWJSON_STREAM_TYPE_STRING:
            printf("string key<%s> value<%s>\n", key_name, data_buf);
        break;
        case LWJSON_STREAM_TYPE_TRUE:
            printf("true key<%s>\n", key_name);
        break;
        case LWJSON_STREAM_TYPE_FALSE:
            printf("false key<%s>\n", key_name);
        break;
        case LWJSON_STREAM_TYPE_NULL:
            printf("null key<%s>\n", key_name);
        break;
        case LWJSON_STREAM_TYPE_NUMBER: {
            printf("number key<%s> value<%s>\n", key_name, data_buf);
        } break;
        case LWJSON_STREAM_TYPE_NONE:
        case LWJSON_STREAM_TYPE_KEY:
        case LWJSON_STREAM_TYPE_OBJECT:
        case LWJSON_STREAM_TYPE_OBJECT_END:
        case LWJSON_STREAM_TYPE_ARRAY:
        case LWJSON_STREAM_TYPE_ARRAY_END:
        default:
        break;
    }
}

UINT WebServer::postRequestJson(
        NX_HTTP_SERVER *server_ptr, 
        UINT request_type, 
        CHAR *resource, 
        NX_PACKET *packet_ptr,
        lwjson_stream_parser_callback_fn callback) {

    ULONG contentLength = 0;
    UINT status = nx_http_server_packet_content_find(server_ptr, &packet_ptr, &contentLength);
    if (status || contentLength <= 0) {
        nx_packet_release(packet_ptr);
        nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_INTERNAL_ERROR, sizeof(NX_HTTP_STATUS_INTERNAL_ERROR)-1, NX_NULL, 0, NX_NULL, 0);
        return(NX_HTTP_CALLBACK_COMPLETED);
    }
    lwjson_stream_parser_t stream_parser;
    lwjson_stream_init(&stream_parser, callback);
    ULONG contentOffset = 0;
    bool done = false;
    do {
        UCHAR *buf = packet_ptr->nx_packet_prepend_ptr;
        ULONG len = packet_ptr->nx_packet_length;
        for (size_t c = 0; c < len; c++) {
            lwjsonr_t res = lwjson_stream_parse(&stream_parser, buf[c]);
            if (res == lwjsonSTREAMINPROG ||
                res == lwjsonSTREAMWAITFIRSTCHAR) {
                // NOP
            } else if (res == lwjsonSTREAMDONE) {
                break;
                done = true;
            } else {
                nx_packet_release(packet_ptr);
                nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_BAD_REQUEST, sizeof(NX_HTTP_STATUS_BAD_REQUEST)-1, NX_NULL, 0, NX_NULL, 0);
                return(NX_HTTP_CALLBACK_COMPLETED);
            }
        }
        contentOffset += len;
        if (!done) {
            done = contentOffset >= contentLength;
        }
        if (!done) {
            nx_packet_release(packet_ptr);
            status = nx_tcp_socket_receive(&(server_ptr -> nx_http_server_socket), &packet_ptr, NX_HTTP_SERVER_TIMEOUT_RECEIVE);
            if (status) {
                nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_REQUEST_TIMEOUT, sizeof(NX_HTTP_STATUS_REQUEST_TIMEOUT)-1, NX_NULL, 0, NX_NULL, 0);
                return(NX_HTTP_CALLBACK_COMPLETED);
            }
        }
    } while (!done);
    nx_packet_release(packet_ptr);
    nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_OK, sizeof(NX_HTTP_STATUS_OK)-1, NX_NULL, 0, NX_NULL, 0);
    return(NX_HTTP_CALLBACK_COMPLETED);
}

#endif  // #ifndef BOOTLOADER

UINT WebServer::requestNotifyCallback(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr) {
    return WebServer::instance().requestNotify(server_ptr, request_type, resource, packet_ptr);
}

UINT WebServer::requestNotify(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr) {
#ifndef BOOTLOADER
    if (strcmp(resource, "/settings") == 0) {
        switch(request_type) {
            case NX_HTTP_SERVER_GET_REQUEST: {
                nx_packet_release(packet_ptr);
                nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_OK, sizeof(NX_HTTP_STATUS_OK)-1, NX_NULL, 0, NX_NULL, 0);
                return(NX_HTTP_CALLBACK_COMPLETED);
            }
            case NX_HTTP_SERVER_POST_REQUEST:
            case NX_HTTP_SERVER_PUT_REQUEST: {
                return postRequestJson(server_ptr, 
                    request_type, 
                    resource, 
                    packet_ptr,
                    jsonStreamSettingsCallback);
            }
        }
    }
#endif  // #ifndef BOOTLOADER
    return(NX_SUCCESS);
}

uint8_t *WebServer::setup(uint8_t *pointer) {
    UINT status;

    const size_t http_server_stack_size = 8192;

    fx_system_initialize();

    status = nx_http_server_create(&http_server, "WebServer", Network::instance().ip(), &ram_disk, pointer, http_server_stack_size, Network::instance().pool(), NX_NULL, requestNotifyCallback);
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

    nx_http_server_mime_maps_additional_set(&http_server, map, 4);

    return pointer;

fail:
    while(1) {}
}

#ifndef BOOTLOADER
__attribute__((section(".text#")))
#include "fs.h"
#else  // #ifndef BOOTLOADER
__attribute__((section(".text#")))
#include "fsbl.h"
#endif  // #ifndef BOOTLOADER

static VOID  _fx_ro_ram_driver(FX_MEDIA *media_ptr)
{
    switch (media_ptr -> fx_media_driver_request)
    {
    case FX_DRIVER_READ: {
        UCHAR *source_buffer =  
            ((UCHAR *)media_ptr -> fx_media_driver_info) +
                    ((media_ptr -> fx_media_driver_logical_sector + 
                      media_ptr -> fx_media_hidden_sectors) * 
                      media_ptr -> fx_media_bytes_per_sector);
        _fx_utility_memory_copy(source_buffer, 
                                media_ptr -> fx_media_driver_buffer, 
                                media_ptr -> fx_media_driver_sectors * 
                                media_ptr -> fx_media_bytes_per_sector);
        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
        break;
    }

    case FX_DRIVER_FLUSH: {
        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
        break;
    }

    case FX_DRIVER_ABORT: {
        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
        break;
    }

    case FX_DRIVER_INIT: {
        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
        break;
    }

    case FX_DRIVER_UNINIT: {
        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
        break;
    }

    case FX_DRIVER_BOOT_READ: {
        UCHAR *source_buffer =  (UCHAR *)media_ptr -> fx_media_driver_info;
        if ( (source_buffer[0] != (UCHAR)0xEB)  ||
            ((source_buffer[1] != (UCHAR)0x34)  &&
             (source_buffer[1] != (UCHAR)0x76)) ||          /* exFAT jump code.  */
             (source_buffer[2] != (UCHAR)0x90)) {
            media_ptr -> fx_media_driver_status =  FX_MEDIA_INVALID;
            return;
        }
        UINT bytes_per_sector =  _fx_utility_16_unsigned_read(&source_buffer[FX_BYTES_SECTOR]);
#ifdef FX_ENABLE_EXFAT
        if (bytes_per_sector == 0 && (source_buffer[1] == (UCHAR)0x76)) {
            bytes_per_sector = (UINT)(1 << source_buffer[FX_EF_BYTE_PER_SECTOR_SHIFT]);
        }
#endif /* FX_ENABLE_EXFAT */
        if (bytes_per_sector > media_ptr -> fx_media_memory_size) {
            media_ptr -> fx_media_driver_status =  FX_BUFFER_ERROR;
            break;
        }
        _fx_utility_memory_copy(source_buffer, media_ptr -> fx_media_driver_buffer,
                                bytes_per_sector);
        media_ptr -> fx_media_driver_status =  FX_SUCCESS;
        break;
    }
    default: {
        media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
        break;
    }
    }
}

bool WebServer::start() {
    UINT status;

    status = fx_media_open(&ram_disk, "APROM Disk", _fx_ro_ram_driver, fs_data, media_memory, sizeof(media_memory));
    if (status) {
        return false;
    }

    status = nx_http_server_start(&http_server);
    if (status) {
        return false;
    }

    return true;
}

#pragma GCC diagnostic pop
