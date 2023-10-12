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
#include "webserver.h"
#include "network.h"
#include "settingsdb.h"

#include "synopGMAC_Dev.h"
#include "nx_m460_eth_driver.h"

#ifndef BOOTLOADER
#include "lwjson/lwjson.h"
#endif  // #ifndef BOOTLOADER

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

    // Top level values only, no nesting
    if (jsp->stack_pos != 2) {
        return;
    }

    const char *key_name = jsp->stack[jsp->stack_pos-1].meta.name;
    const char *data_buf = jsp->data.str.buff;

    switch(type) {
        case LWJSON_STREAM_TYPE_STRING:
            SettingsDB::instance().setString(key_name, data_buf);
        break;
        case LWJSON_STREAM_TYPE_TRUE:
            SettingsDB::instance().setBool(key_name, true);
        break;
        case LWJSON_STREAM_TYPE_FALSE:
            SettingsDB::instance().setBool(key_name, false);
        break;
        case LWJSON_STREAM_TYPE_NULL:
            SettingsDB::instance().setNull(key_name);
        break;
        case LWJSON_STREAM_TYPE_NUMBER: {
            SettingsDB::instance().setNumber(key_name, strtof(data_buf, NULL));
        } break;
        case LWJSON_STREAM_TYPE_NONE:
        case LWJSON_STREAM_TYPE_KEY:
        case LWJSON_STREAM_TYPE_OBJECT:
        case LWJSON_STREAM_TYPE_OBJECT_END:
        case LWJSON_STREAM_TYPE_ARRAY:
        case LWJSON_STREAM_TYPE_ARRAY_END:
        default:
        // not supported
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
    if (status) {
        nx_packet_release(packet_ptr);
        nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_REQUEST_TIMEOUT, sizeof(NX_HTTP_STATUS_REQUEST_TIMEOUT)-1, NX_NULL, 0, NX_NULL, 0);
        return(NX_HTTP_CALLBACK_COMPLETED);
    }
    if (contentLength == 0) {
        nx_packet_release(packet_ptr);
        nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_NO_CONTENT, sizeof(NX_HTTP_STATUS_NO_CONTENT)-1, NX_NULL, 0, NX_NULL, 0);
        return(NX_HTTP_CALLBACK_COMPLETED);
    }
    lwjson_stream_parser_t stream_parser;
    lwjson_stream_init(&stream_parser, callback);
    ULONG contentOffset = 0;
    bool done = false;
    do {
        UCHAR *jsonBuf = packet_ptr->nx_packet_prepend_ptr;
        ULONG jsonLen = packet_ptr->nx_packet_length;
        for (size_t c = 0; c < jsonLen; c++) {
            lwjsonr_t res = lwjson_stream_parse(&stream_parser, jsonBuf[c]);
            if (res == lwjsonSTREAMINPROG ||
                res == lwjsonSTREAMWAITFIRSTCHAR ||
                res == lwjsonOK) {
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
        contentOffset += jsonLen;
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

#ifdef BOOTLOADER
UINT WebServer::postRequestUpload(
        NX_HTTP_SERVER *server_ptr, 
        UINT request_type, 
        CHAR *resource, 
        NX_PACKET *packet_ptr) {

    ULONG offset = 0, chunk_length = 0, total_length = 0;
    UCHAR buffer[MAX_ETHERNET_PAYLOAD + 1]; // plus 1 for null termination
    while(nx_http_server_get_entity_header(server_ptr, &packet_ptr, buffer, sizeof(buffer)) == NX_SUCCESS) {
        buffer[chunk_length] = 0;
        if (strstr((const char *)buffer, "application/octet-stream") != NULL) {
            while(nx_http_server_get_entity_content(server_ptr, &packet_ptr, &offset, &chunk_length) == NX_SUCCESS) {
                nx_packet_data_extract_offset(packet_ptr, offset, buffer, chunk_length, &chunk_length);
                total_length += chunk_length;
            }
        }
    }

    nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_OK, sizeof(NX_HTTP_STATUS_OK)-1, NX_NULL, 0, NX_NULL, 0);
    return(NX_HTTP_CALLBACK_COMPLETED);
}
#endif  // #ifdef BOOTLOADER

UINT WebServer::requestNotifyCallback(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr) {
    return WebServer::instance().requestNotify(server_ptr, request_type, resource, packet_ptr);
}

UINT WebServer::requestNotify(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr) {
    switch(request_type) {
        case NX_HTTP_SERVER_GET_REQUEST: {
#ifndef BOOTLOADER
            if (strcmp(resource, "/settings") == 0) {
                SettingsDB::instance().dump();
                nx_packet_release(packet_ptr);
                nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_OK, sizeof(NX_HTTP_STATUS_OK)-1, NX_NULL, 0, NX_NULL, 0);
                return(NX_HTTP_CALLBACK_COMPLETED);
            }
#endif  // #ifndef BOOTLOADER
        } break;
        case NX_HTTP_SERVER_POST_REQUEST:
        case NX_HTTP_SERVER_PUT_REQUEST: {
#ifndef BOOTLOADER
            if (strcmp(resource, "/settings") == 0) {
                return postRequestJson(server_ptr, 
                    request_type, 
                    resource, 
                    packet_ptr,
                    jsonStreamSettingsCallback);
            }
#endif  // #ifndef BOOTLOADER
#ifdef BOOTLOADER
            if (strcmp(resource, "/upload") == 0) {
                return postRequestUpload(server_ptr, 
                    request_type, 
                    resource, 
                    packet_ptr);
            }
#endif  // #ifdef BOOTLOADER
            nx_packet_release(packet_ptr);
            nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_METHOD_NOT_ALLOWED, sizeof(NX_HTTP_STATUS_METHOD_NOT_ALLOWED)-1, NX_NULL, 0, NX_NULL, 0);
            return(NX_HTTP_CALLBACK_COMPLETED);
        } break;
        case NX_HTTP_SERVER_HEAD_REQUEST:
        case NX_HTTP_SERVER_DELETE_REQUEST: {
            nx_packet_release(packet_ptr);
            nx_http_server_callback_response_send_extended(server_ptr, NX_HTTP_STATUS_METHOD_NOT_ALLOWED, sizeof(NX_HTTP_STATUS_METHOD_NOT_ALLOWED)-1, NX_NULL, 0, NX_NULL, 0);
            return(NX_HTTP_CALLBACK_COMPLETED);
        } break;
    }
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

void WebServer::APROMDiskDriver(FX_MEDIA *media_ptr)
{
    switch (media_ptr -> fx_media_driver_request)
    {
    case FX_DRIVER_READ: {
        UCHAR *source_buffer =  
            ((UCHAR *)media_ptr->fx_media_driver_info) +
                    ((media_ptr->fx_media_driver_logical_sector + 
                      media_ptr->fx_media_hidden_sectors) * 
                      media_ptr->fx_media_bytes_per_sector);
        _fx_utility_memory_copy(source_buffer, 
                                media_ptr->fx_media_driver_buffer, 
                                media_ptr->fx_media_driver_sectors * 
                                media_ptr->fx_media_bytes_per_sector);
        media_ptr->fx_media_driver_status =  FX_SUCCESS;
        break;
    }

    case FX_DRIVER_FLUSH: {
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;
    }

    case FX_DRIVER_ABORT: {
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;
    }

    case FX_DRIVER_INIT: {
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;
    }

    case FX_DRIVER_UNINIT: {
        media_ptr->fx_media_driver_status = FX_SUCCESS;
        break;
    }

    case FX_DRIVER_BOOT_READ: {
        UCHAR *source_buffer = (UCHAR *)media_ptr->fx_media_driver_info;
        if ( (source_buffer[0] != (UCHAR)0xEB)  ||
            ((source_buffer[1] != (UCHAR)0x34)  &&
             (source_buffer[1] != (UCHAR)0x76)) ||          /* exFAT jump code.  */
             (source_buffer[2] != (UCHAR)0x90)) {
            media_ptr->fx_media_driver_status = FX_MEDIA_INVALID;
            return;
        }
        UINT bytes_per_sector = _fx_utility_16_unsigned_read(&source_buffer[FX_BYTES_SECTOR]);
#ifdef FX_ENABLE_EXFAT
        if (bytes_per_sector == 0 && (source_buffer[1] == (UCHAR)0x76)) {
            bytes_per_sector = (UINT)(1 << source_buffer[FX_EF_BYTE_PER_SECTOR_SHIFT]);
        }
#endif /* FX_ENABLE_EXFAT */
        if (bytes_per_sector > media_ptr -> fx_media_memory_size) {
            media_ptr -> fx_media_driver_status =  FX_BUFFER_ERROR;
            break;
        }
        _fx_utility_memory_copy(source_buffer, media_ptr -> fx_media_driver_buffer, bytes_per_sector);
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

#ifdef BOOTLOADER
static __attribute__((section(".text#")))
#include "fsbl.h"
#else  // #ifndef BOOTLOADER
static __attribute__((section(".text#")))
#include "fs.h"
#endif  // #ifndef BOOTLOADER

    status = fx_media_open(&ram_disk, "APROM Disk", APROMDiskDriver, fs_data, media_memory, sizeof(media_memory));
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
