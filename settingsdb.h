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
