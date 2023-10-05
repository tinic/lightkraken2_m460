#include "fx_api.h"
#include "../build/fs.h"

extern void _fx_ram_driver(FX_MEDIA *media_ptr);
static unsigned char media_memory[512] = { 0 };
static FX_MEDIA ram_disk = { 0 };

int main() {
    fx_system_initialize();

    UINT status =  fx_media_open(&ram_disk, "RAM Disk", _fx_ram_driver, fs_data, media_memory, sizeof(media_memory));
    if (status) {
        return 0;
    }

    return 0;
}
