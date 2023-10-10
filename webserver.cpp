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

uint8_t *WebServer::setup(uint8_t *pointer) {
    UINT status;

    const size_t http_server_stack_size = 2048;

    fx_system_initialize();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

    status = nx_http_server_create(&http_server, "WebServer", Network::instance().ip(), &ram_disk, pointer, http_server_stack_size, Network::instance().pool(), NX_NULL, NX_NULL);
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

    status = fx_media_open(&ram_disk, "APROM Disk", _fx_ro_ram_driver, fs_data, media_memory, sizeof(media_memory));
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
