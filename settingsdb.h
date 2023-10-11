#ifndef _SETTINGSDB_H_
#define _SETTINGSDB_H_

#include <flashdb.h>

class SettingsDB {
public:
    static SettingsDB &instance();

    size_t getString(const char *key, char *value, size_t maxlen);
    bool getBool(const char *key, bool *value);
    bool getNumber(const char *key, float *value);
    bool getNull(const char *key);

    void setString(const char *key, const char *str);
    void setBool(const char *key, bool value);
    void setNumber(const char *key, float value);
    void setNull(const char *key);

    void dump();

private:
    void init();
    bool initialized = false;

    static void lock();
    static void unlock();

    struct fdb_kvdb kvdb {};
};

#endif  // #ifndef _SETTINGSDB_H_
