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
#include "network.h"
#include "webserver.h"
#include "settingsdb.h"

static TX_THREAD thread_startup {};
void thread_startup_entry(ULONG thread_input) {
    NX_PARAMETER_NOT_USED(thread_input);

    SettingsDB::instance().dump();

    if (!Network::instance().start()) {
        return;
    }

    if (!WebServer::instance().start()) {
        return;
    }

    tx_thread_relinquish();
}

extern "C" void tx_application_define(void *first_unused_memory);
void tx_application_define(void *first_unused_memory) {
    uint8_t *pointer = (uint8_t *)first_unused_memory;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

    const size_t startup_stack_size = 4096;
    tx_thread_create(&thread_startup, "startup", thread_startup_entry, 0, pointer, startup_stack_size, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    pointer = pointer + startup_stack_size;

#pragma GCC diagnostic pop

    pointer = Network::instance().setup(pointer);

    pointer = WebServer::instance().setup(pointer);

    printf("Consumed %d bytes of RAM.\n", (int)(pointer-(uint8_t *)first_unused_memory));
}
