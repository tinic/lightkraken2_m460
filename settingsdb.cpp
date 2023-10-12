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
#include "settingsdb.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wregister"

#include "NuMicro.h"

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstack-usage="
#pragma GCC diagnostic ignored "-Wwrite-strings"

#include "nx_api.h"

#include <alloca.h>
#include <string.h>

SettingsDB &SettingsDB::instance() {
    static SettingsDB settingsDB;
    if (!settingsDB.initialized) {
        settingsDB.initialized = true;
        settingsDB.init();
    }
    return settingsDB;
}

void SettingsDB::erase() {

    lock();

    SYS_UnlockReg();

    FMC_ENABLE_AP_UPDATE();

    size_t addr = FLASH_DB_START_ADDRESS;
    for (size_t i = 0; i < FLASH_DB_LENGTH; i += FLASH_DB_BLOCK_SIZE) {
        printf("Erasing block at %08x\n", addr);
        FMC_Erase_Bank(addr);
        addr += FLASH_DB_BLOCK_SIZE;
    }

    FMC_DISABLE_AP_UPDATE();

    SYS_LockReg();

    unlock();
}

void SettingsDB::init() {

//    erase();

    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_LOCK, (void *)lock);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void *)unlock);

    uint32_t boot_count = 0;

    NXD_ADDRESS last_ipv4 {};
    NXD_ADDRESS last_ipv6 {};

    struct fdb_default_kv default_kv;
    static struct fdb_default_kv_node default_kv_table[] = {
        {"boot_count@i", &boot_count, sizeof(boot_count)},
        {"last_ipv4@a", &last_ipv4, sizeof(last_ipv4)},
        {"last_ipv6@a", &last_ipv6, sizeof(last_ipv6)},
    };
    default_kv.kvs = default_kv_table;
    default_kv.num = sizeof(default_kv_table) / sizeof(default_kv_table[0]);

    fdb_err_t result = fdb_kvdb_init(&kvdb, "env", "fdb_kvdb1", &default_kv, NULL);
    if (result != FDB_NO_ERR) {
        while(1) {}
    }

    struct fdb_blob blob {};
    fdb_kv_get_blob(&kvdb, "boot_count@i", fdb_blob_make(&blob, &boot_count, sizeof(boot_count)));
    boot_count++;
    fdb_kv_set_blob(&kvdb, "boot_count@i", fdb_blob_make(&blob, &boot_count, sizeof(boot_count)));
}

void SettingsDB::lock() {
    __enable_irq();
}

void SettingsDB::unlock() {
    __disable_irq();
}

void SettingsDB::dump() {
    struct fdb_kv_iterator iterator {};

    printf("Dumping database:\n");

    fdb_kv_iterator_init(&kvdb, &iterator);
    while (fdb_kv_iterate(&kvdb, &iterator)) {

        fdb_kv_t cur_kv = &(iterator.curr_kv);
        size_t data_size = (size_t)cur_kv->value_len;
        struct fdb_blob blob {};

        size_t name_len = strlen(cur_kv->name);
        if (name_len > 2 && cur_kv->name[name_len-2] == '@') {
            switch(cur_kv->name[name_len-1]) {
                case 's': {
                    uint8_t *data_buf = (uint8_t *)alloca(data_size + 1);
                    data_buf[data_size] = 0;
                    fdb_blob_read(reinterpret_cast<fdb_db_t>(&kvdb), fdb_kv_to_blob(cur_kv, fdb_blob_make(&blob, data_buf, data_size)));
                    printf("String: <%s> <%s>\n", cur_kv->name, data_buf);
                } break;
                case 'b': {
                    bool value = false;
                    fdb_blob_read(reinterpret_cast<fdb_db_t>(&kvdb), fdb_kv_to_blob(cur_kv, fdb_blob_make(&blob, &value, sizeof(value))));
                    printf("Bool:   <%s> <%s>\n", cur_kv->name, value ? "true" : "false");
                } break;
                case 'f': {
                    float value = 0;
                    fdb_blob_read(reinterpret_cast<fdb_db_t>(&kvdb), fdb_kv_to_blob(cur_kv, fdb_blob_make(&blob, &value, sizeof(value))));
                    //printf("Float:  <%s> <%f>\n", cur_kv->name, double(value));
                } break;
                case 'n': {
                    char value = 0;
                    fdb_blob_read(reinterpret_cast<fdb_db_t>(&kvdb), fdb_kv_to_blob(cur_kv, fdb_blob_make(&blob, &value, sizeof(value))));
                    printf("Null:   <%s>\n", cur_kv->name);
                } break;
                case 'i': {
                    uint32_t value = 0;
                    fdb_blob_read(reinterpret_cast<fdb_db_t>(&kvdb), fdb_kv_to_blob(cur_kv, fdb_blob_make(&blob, &value, sizeof(value))));
                    printf("Int:    <%s> <%d>\n", cur_kv->name, value);
                } break;
                case 'a': {
                    NXD_ADDRESS value {};
                    fdb_blob_read(reinterpret_cast<fdb_db_t>(&kvdb), fdb_kv_to_blob(cur_kv, fdb_blob_make(&blob, &value, sizeof(value))));
                    if (value.nxd_ip_version == NX_IP_VERSION_V4) {
                        printf("Addr:   <%s> <%d::%d::%d::%d>\n", 
                            cur_kv->name, 
                            (value.nxd_ip_address.v4>>24)&0xFF,
                            (value.nxd_ip_address.v4>>16)&0xFF,
                            (value.nxd_ip_address.v4>> 8)&0xFF,
                            (value.nxd_ip_address.v4>> 0)&0xFF);
                    } else if (value.nxd_ip_version == NX_IP_VERSION_V6) {
                        printf("Addr:   <%s> <%08x::%08x::%08x::%08x>\n", 
                            cur_kv->name, 
                            value.nxd_ip_address.v6[0],
                            value.nxd_ip_address.v6[1],
                            value.nxd_ip_address.v6[2],
                            value.nxd_ip_address.v6[3]);
                    } else {
                        printf("Addr:   <%s> <invalid>\n", 
                            cur_kv->name);
                    }
                } break;
                default:
                break;
            }
        }
    }
    printf("Dumping database done.\n");
}

size_t SettingsDB::getString(const char *key, char *value, size_t maxlen) {
    char *keyS = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyS, key);
    strcat(keyS, "@s");
    struct fdb_blob blob {};
    size_t len = 0;
    if ((len = fdb_kv_get_blob(&kvdb, keyS, fdb_blob_make(&blob, reinterpret_cast<const void *>(value), maxlen))) > 0) {
        value[maxlen-1] = 0;
        return len;
    }
    return 0;
}

bool SettingsDB::getBool(const char *key, bool *value) {
    char *keyB = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyB, key);
    strcat(keyB, "@b");
    struct fdb_blob blob {};
    if (fdb_kv_get_blob(&kvdb, keyB, fdb_blob_make(&blob, reinterpret_cast<const void *>(value), sizeof(bool))) == sizeof(bool)) {
        return true;
    }
    return false;
}

bool SettingsDB::getNumber(const char *key, float *value) {
    char *keyF = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyF, key);
    strcat(keyF, "@f");
    struct fdb_blob blob {};
    if (fdb_kv_get_blob(&kvdb, keyF, fdb_blob_make(&blob, reinterpret_cast<const void *>(value), sizeof(float))) == sizeof(float)) {
        return true;
    }
    return false;
}

bool SettingsDB::getNull(const char *key) {
    char *keyN = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyN, key);
    strcat(keyN, "@n");
    char value = 0;
    struct fdb_blob blob {};
    if (fdb_kv_get_blob(&kvdb, keyN, fdb_blob_make(&blob, reinterpret_cast<const void *>(&value), sizeof(char))) == sizeof(char)) {
        return true;
    }
    return false;
}

void SettingsDB::setString(const char *key, const char *str) {
    printf("setString <%s> <%s>\n", key, str);
    char *keyS = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyS, key);
    strcat(keyS, "@s");
    fdb_kv_set(&kvdb, keyS, str);
}

void SettingsDB::setBool(const char *key, bool value) {
    printf("setBool <%s> <%s>\n", key, value ? "true" : "false");
    char *keyB = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyB, key);
    strcat(keyB, "@b");
    struct fdb_blob blob {};
    fdb_kv_set_blob(&kvdb, keyB, fdb_blob_make(&blob, reinterpret_cast<const void *>(&value), sizeof(value)));
}

void SettingsDB::setNumber(const char *key, float value) {
    //printf("setNumber <%s> <%f>\n", key, double(value));
    char *keyF = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyF, key);
    strcat(keyF, "@f");
    struct fdb_blob blob {};
    fdb_kv_set_blob(&kvdb, keyF, fdb_blob_make(&blob, reinterpret_cast<const void *>(&value), sizeof(value)));
}

void SettingsDB::setNull(const char *key) {
    printf("setNull <%s>\n", key);
    char *keyN = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyN, key);
    strcat(keyN, "@n");
    char value = 0;
    struct fdb_blob blob {};
    fdb_kv_set_blob(&kvdb, keyN, fdb_blob_make(&blob, reinterpret_cast<const void *>(&value), sizeof(value)));
}
