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
#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <stdint.h>

#include "tx_api.h"
#include "nx_api.h"
#include "nx_auto_ip.h"
#include "nxd_dhcp_client.h"

class Network {
public:
    static Network &instance();

    uint8_t *setup(uint8_t *pointer);
    bool start();

    NX_IP *ip() { return &client_ip; };
    NX_PACKET_POOL *pool() { return &client_pool; }

private:

    void init();
    bool initialized = false;

    NX_IP client_ip {};
    NX_AUTO_IP auto_ip {};
    NX_DHCP dhcp_client {};
    NX_PACKET_POOL client_pool {};
};

#endif  // #ifndef _NETWORK_H_
