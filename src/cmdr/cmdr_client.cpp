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

#include "cmdr_client.h"

#include "../math/_math.h"

#include <netdb.h>
#include <fcntl.h>
#include <pthread.h>

#include <string.h>
#include <iostream>
#include <stdlib.h>

cmdr::cmdr_client::cmdr_client(const char* hostname, int port,
        const char* pin, const char* partner_pin)
{
    socket = new hbc::socket_client(hostname, port);
    socket->acknak = &cmdr::acknak;
    socket->acknak_cookie = this;

    this->pin = pin;
    this->partner_pin = partner_pin;
    data_channel = 0;

    connected_callback = 0;
    connected_callback_cookie = 0;

    frame_index_callback = 0;
    frame_index_callback_cookie = 0;
}

cmdr::cmdr_client::~cmdr_client()
{
    if (socket) delete socket;
    if (data_channel) ::close(data_channel);
}

bool cmdr::cmdr_client::start()
{
    if (socket->open() != hbc::SOCK_OK) return false;

    pthread_t pthread;
    pthread_create(&pthread, 0, &cmdr::socket_start, socket);

    fprintf(stdout, "[%d] Mapping command channel\n", socket->fd);
    int blen = (int) strlen(pin) + 2;
    unsigned char b[blen];
    b[0] = (int) command;
    b[1] = (int) strlen(pin);
    memcpy(&b[2], pin, strlen(pin));
    cmdr::write(socket->fd, cmdr_mapping, b, blen);

    return true;
}

void cmdr::cmdr_client::disconnect()
{
    socket->disconnect();
    close(data_channel);
}

hbc::socket_packet* cmdr::cmdr_client::build_last_frame_index(int frame_index)
{
    int length = 6;
    unsigned char* payload = (unsigned char*) malloc(length * sizeof(unsigned char));
    payload[0] = cmdr_partner_pipe;
    payload[1] = cmdr_frame_index;
    hbc::int_to_bytes(frame_index, &payload[2]);

    hbc::socket_packet* packet = (hbc::socket_packet*) malloc(sizeof(hbc::socket_packet));
    packet->sock = socket;
    packet->payload = payload;
    packet->length = length;
    return packet;
}

void cmdr::write(int fd, char key, const unsigned char* payload, int length)
{
    int burst_length = length + 1;

    unsigned char buffer[burst_length];
    buffer[0] = key;
    memcpy(&buffer[1], payload, length);

    hbc::write(fd, buffer, burst_length);
}

void* cmdr::socket_start(void* cookie)
{
    hbc::socket_client* socket = (hbc::socket_client*) cookie;
    socket->start();
    return 0;
}

const char* connection_type_str(cmdr::connection_type type)
{
    switch (type)
    {
        case cmdr::unknown:
            return "unknown";
        case cmdr::command:
            return "command";
        case cmdr::data:
            return "data";
        default:
            return "bad";
    }
}

const char* connect_to_pin_str(cmdr::connect_to_pin_response type)
{
    switch (type)
    {
        case cmdr::connected:
            return "connected";
        case cmdr::waiting_for_partner:
            return "waiting for partner";
        case cmdr::failed:
            return "failed";
        default:
            return "bad response";
    }
}

void cmdr::cmdr_client::map_data_channel(hbc::socket_client* socket)
{
    hbc::open(socket->hostname, socket->port, &data_channel);

    fprintf(stdout, "[%d] Mapping data channel\n", socket->fd);
    int blen = (int) strlen(pin) + 2;
    unsigned char b[blen];
    b[0] = (int) data;
    b[1] = (int) strlen(pin);
    memcpy(&b[2], pin, strlen(pin));
    write(data_channel, cmdr_mapping, b, blen);
}

int cmdr::acknak(hbc::socket_packet* packet, void* cookie)
{
    cmdr_client* client = (cmdr_client*) cookie;
    hbc::socket_client* socket = (hbc::socket_client*) packet->sock;

    int key = packet->payload[0];
    unsigned char* value = packet->length > 1 ? &packet->payload[1] : 0;
    int value_len = packet->length - 1;

    int ack = 1;

    if (key == cmdr_mapping_r)
    {
        connection_type type = (connection_type) value[0];

        fprintf(stdout, "[%d] Mapped as type: %s (%d)\n",
        socket->fd, connection_type_str(type), type);

        if (type == command)
        {
            client->map_data_channel(socket);
        }
        else if (type == data)
        {
            fprintf(stdout, "[%d] Connecting to partner: %s\n", socket->fd, client->partner_pin);
            cmdr::write(socket->fd, cmdr_connect_to,
                    (unsigned char*) client->partner_pin, strlen(client->partner_pin));
        }
    }
    else if (key == cmdr_connect_to_r)
    {
        connect_to_pin_response type = (connect_to_pin_response) value[0];
        fprintf(stdout, "[%d] Partner connection response: %s (%d)\n",
                socket->fd, connect_to_pin_str(type), type);

        if (type == connected)
        {
            client->connected_callback(client->connected_callback_cookie);
        }
    }
    else if (key == cmdr_partner_disconnected)
    {
        fprintf(stdout, "[%d] Partner disconnected\n", socket->fd);

        ::close(client->data_channel);
        client->data_channel = 0;

        client->map_data_channel(socket);
    }
    else if (key == cmdr_frame_index)
    {
        int frame_index = hbc::bytes_to_int(value);
        client->frame_index_callback(frame_index, client->frame_index_callback_cookie);
    }

    fflush(stdout);

    return ack;
}

