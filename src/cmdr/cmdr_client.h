/**
 * Copyright (c) 2012 Martin M Reed
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CMDR_CLIENT_H
#define CMDR_CLIENT_H

#include "packet_keys.h"
#include "../socket/socket_client.h"

#undef OSX_PLATFORM

#define printError(x) fprintf(stderr,"Bad Wolf[%s]\n", x)

namespace cmdr
{

void write(int fd, char key, const unsigned char* payload, int length);

int acknak(hbc::socket_packet* packet, void* cookie);
void* socket_start(void* cookie);

class cmdr_client
{
    friend int acknak(hbc::socket_packet* packet, void* cookie);

public:

    cmdr_client(const char* hostname, int port, const char* pin, const char* connect_to_pin);
    virtual ~cmdr_client();

    bool start();
    void disconnect();

    int data_channel;

    void (*connected_callback)(void* cookie);
    void *connected_callback_cookie;

    void (*frame_index_callback)(int index, void* cookie);
    void *frame_index_callback_cookie;

    hbc::socket_packet* build_last_frame_index(int frame_index);

private:

    void map_data_channel(hbc::socket_client* socket);

    hbc::socket_client* socket;
    const char* pin;
    const char* partner_pin;
};

}

#endif
