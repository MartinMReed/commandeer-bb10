/**
 * Copyright (c) 2009-2012 Martin M Reed
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

#include "socket.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>

#include "../math/_math.h"

int hbc::is_open(int fd)
{
    return fd && (fcntl(fd, F_GETFL) != -1 || errno != EBADF);
}

void hbc::free_packet(socket_packet* packet)
{
    free(packet->payload);
    free(packet);
}

hbc::socket_error hbc::open(const char* hostname, int port, int* fd)
{
    // open a socket to listen on
    int _fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (_fd < 0) return SOCK_CANNOT_OPEN;

    hostent* host = gethostbyname(hostname);
    if (!host) return SOCK_HOST_NOT_FOUND;

    sockaddr_in s_addr;
    bzero((char*) &s_addr, sizeof(s_addr));

    s_addr.sin_family = AF_INET;
    bcopy((char*) host->h_addr, (char*) &s_addr.sin_addr.s_addr, host->h_length);
    s_addr.sin_port = htons(port);

    if (::connect(_fd, (sockaddr*) &s_addr, sizeof(s_addr)) < 0) return SOCK_CANNOT_CONNECT;

    *fd = _fd;

    return SOCK_OK;
}

hbc::socket::socket()
{
    fd = 0;
    adopted = false;
    read_buffer = (unsigned char*) malloc((BURST_MAX + 1) * sizeof(unsigned char));
}

hbc::socket::~socket()
{
    if (!adopted)
    {
        disconnect();
    }

    free(read_buffer);
}

void hbc::socket::disconnect()
{
    if (fd)
    {
        close(fd);
        fd = 0;
    }
}

/**
 * Write a payload out to the given socket.
 *
 * @param fd The socket to write to.
 * @param payload The packet-burst payload.
 * @param length The byte count of the payload parameter.
 */
int hbc::socket::write(const unsigned char* payload, int length)
{
    return hbc::write(fd, payload, length);
}

int hbc::write(int fd, const unsigned char* payload, int length)
{
    if (length > PAYLOAD_MAX)
    {
        int buffer_length = num_digits(length);
        unsigned char buffer[buffer_length];
        sprintf((char*) buffer, "%i", length);
        int w = hbc::write_packet(fd, buffer, length);
        if (w < 0) return w;
    }

    int o = 0;
    do
    {
        int w = hbc::write_packet(fd, &payload[o], length - o);
        if (w < 0) return w;
        o += w;
    }
    while (o < length);

//    std::string pstr(reinterpret_cast<const char*>(payload), length);
//    fprintf(stdout, "<- fd[%i], chunked[%i], %s\n", fd, length > PAYLOAD_MAX, pstr.c_str());

    return o;
}

int hbc::write_packet(int fd, const unsigned char* payload, int length)
{
    unsigned char write_buffer[socket_burst_EOH];

    // if we add PAYLOAD_MAX to the length of what we have already sent
    // and it is greater than the total length, that means we are at
    // the last packet and can send less than PAYLOAD_MAX.
    int payload_size = PAYLOAD_MAX > length ? length : PAYLOAD_MAX;
    write_buffer[socket_burst_size] = payload_size;

    // if we add the current burst length to what we have already sent,
    // and it is less than the total length, that means we still have
    // more packets to send, so set the chunked flag to tell the recipient
    // to keep reading our other packets.
    write_buffer[socket_burst_chunked] = payload_size < length;

    int w = hbc::write_all(fd, write_buffer, socket_burst_EOH);
    if (w < 0) return w;

    w = hbc::write_all(fd, payload, payload_size);
    if (w < 0) return w;

    return payload_size;
}

int hbc::write_all(int fd, const unsigned char* payload, int length)
{
    int o = 0;
    do
    {
        int w = ::write(fd, &payload[o], (length - o) * sizeof(unsigned char));
        if (w < 0) return w;
        o += w;
    }
    while (o < length);
    return o;
}

int hbc::socket::read_packet()
{
    int o = ::read(fd, read_buffer, sizeof(unsigned char));
    if (o <= 0) return o;

    int packet_size = read_buffer[socket_burst_size] + socket_burst_EOH;

    do
    {
        int r = ::read(fd, &read_buffer[o], (packet_size - o) * sizeof(unsigned char));
        if (r <= 0) return r;
        o += r;
    }
    while (o < packet_size);

    // we keep an extra char at the end, so top it off
    // with a NULL in case we need to use it as a string
    read_buffer[o] = 0;

    return o;
}

/**
 * Read from the given socket.
 *
 * @return true if the Client/Server should continue
 * their conversation.
 */
int hbc::socket::read()
{
    int r = read_packet();

    if (r <= 0) return 0;
    else if (r != read_buffer[socket_burst_size] + socket_burst_EOH)
    {
        fprintf(stderr, "did not read[%d] full[%d+%d] socket burst! something went wrong... bailing on client\n",
                r, (int) read_buffer[socket_burst_size], socket_burst_EOH);
        return 0;
    }

    // if the bursts are chunked, the first burst will be
    // the total size of all the chunks combined
    int payload_size;
    if (!read_buffer[socket_burst_chunked]) payload_size = read_buffer[socket_burst_size];
    else payload_size = atoi(reinterpret_cast<const char*>(&read_buffer[socket_burst_EOH]));

    unsigned char* payload = (unsigned char*) malloc((payload_size + 1) * sizeof(unsigned char));
    payload[payload_size] = 0;

    if (!read_buffer[socket_burst_chunked])
    {
        memcpy(payload, &read_buffer[socket_burst_EOH], payload_size);
    }
    else
    {
        int o = 0;

        // continue reading until the last chunk as been sent.
        // the last chunk will specify a 0 flag in the chunk flag
        // which will disable this loop.
        do
        {
            int r = read_packet();

            if (r <= 0)
            {
                free(payload);
                return 0;
            }

            memcpy(&payload[o], &read_buffer[socket_burst_EOH], read_buffer[socket_burst_size]);
            o += read_buffer[socket_burst_size];
        }
        while (read_buffer[socket_burst_chunked]);

        if (o != payload_size)
        {
            free(payload);
            fprintf(stderr, "did not read[%d] full[%d] payload! something went wrong... bailing on client\n",
                    o, payload_size);
            return 0;
        }
    }

//    fprintf(stdout, "-> fd[%i], %s\n", fd, payload);

    socket_packet* packet = (socket_packet*) malloc(sizeof(socket_packet));
    packet->payload = payload;
    packet->length = payload_size;
    packet->sock = this;

    return acknak(packet, acknak_cookie);

}
