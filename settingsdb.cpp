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
