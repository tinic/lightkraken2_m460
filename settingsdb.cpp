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

void SettingsDB::init() {
    fdb_err_t result = fdb_kvdb_init(&kvdb, "env", "fdb_kvdb1", NULL, NULL);
    if (result != FDB_NO_ERR) {
        while(1) {}
    }

    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_LOCK, (void *)lock);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void *)unlock);
}

void SettingsDB::lock() {
    __enable_irq();
}

void SettingsDB::unlock() {
    __disable_irq();
}

void SettingsDB::dump() {
    struct fdb_kv_iterator iterator {};

    fdb_kv_iterator_init(&kvdb, &iterator);
    while (fdb_kv_iterate(&kvdb, &iterator)) {

        fdb_kv_t cur_kv = &(iterator.curr_kv);
        //size_t data_size = (size_t)cur_kv->value_len;

        printf("%s\n", cur_kv->name);
        size_t name_len = strlen(cur_kv->name);
        if (name_len > 2 && cur_kv->name[name_len-2] == '@') {
            switch(cur_kv->name[name_len-1]) {
                case 's':
                case 'b':
                case 'f':
                case 'n':
                break;
                default:
                break;
            }
        }

/*      rt_strncpy(kv_tbl[index].name, cur_kv->name, 32);
        kv_tbl[index].saved_data_size = data_size;
        kv_tbl[index].addr = cur_kv->addr.start;
        struct fdb_blob fdb_blob;
        fdb_blob_read((fdb_db_t)db, fdb_kv_to_blob(cur_kv, fdb_blob_make(&fdb_blob, &kv_tbl[index].value, data_size)));
*/
    }
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
    char *keyS = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyS, key);
    strcat(keyS, "@s");
    fdb_kv_set(&kvdb, keyS, str);
}

void SettingsDB::setBool(const char *key, bool value) {
    char *keyB = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyB, key);
    strcat(keyB, "@b");
    struct fdb_blob blob {};
    fdb_kv_set_blob(&kvdb, keyB, fdb_blob_make(&blob, reinterpret_cast<const void *>(&value), sizeof(value)));
}

void SettingsDB::setNumber(const char *key, float value) {
    char *keyF = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyF, key);
    strcat(keyF, "@f");
    struct fdb_blob blob {};
    fdb_kv_set_blob(&kvdb, keyF, fdb_blob_make(&blob, reinterpret_cast<const void *>(&value), sizeof(value)));
}

void SettingsDB::setNull(const char *key) {
    char *keyN = reinterpret_cast<char *>(alloca(strlen(key)+2));
    strcpy(keyN, key);
    strcat(keyN, "@n");
    char value = 0;
    struct fdb_blob blob {};
    fdb_kv_set_blob(&kvdb, keyN, fdb_blob_make(&blob, reinterpret_cast<const void *>(&value), sizeof(value)));
}
