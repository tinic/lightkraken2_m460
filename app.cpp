#include "network.h"
#include "webserver.h"

static TX_THREAD thread_startup {};
void thread_startup_entry(ULONG thread_input) {
    NX_PARAMETER_NOT_USED(thread_input);

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

    const size_t startup_stack_size = 1024;
    tx_thread_create(&thread_startup, "startup", thread_startup_entry, 0, pointer, startup_stack_size, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    pointer = pointer + startup_stack_size;

#pragma GCC diagnostic pop

    pointer = Network::instance().setup(pointer);

    pointer = WebServer::instance().setup(pointer);

    printf("Consumed %d bytes of RAM.\n", (int)(pointer-(uint8_t *)first_unused_memory));
}
