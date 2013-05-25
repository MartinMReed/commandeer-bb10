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

#include "packet_sender.h"

cmdr::packet_sender::packet_sender()
{
    pthread_mutex_init(&m_mutex, 0);
    pthread_cond_init(&m_cond, 0);
}

void cmdr::packet_sender::start()
{
    if (m_running) return;
    m_running = true;

    pthread_t pthread;
    pthread_create(&pthread, 0, &cmdr::sending_thread, this);
}

void cmdr::packet_sender::send(hbc::socket_packet* packet)
{
    if (!m_running) start();

    m_packets.push_back(packet);
    pthread_cond_signal(&m_cond);
}

void cmdr::packet_sender::run()
{
    while (m_running)
    {
        if (m_packets.empty())
        {
            pthread_mutex_lock(&m_mutex);
            pthread_cond_wait(&m_cond, &m_mutex);
            pthread_mutex_unlock(&m_mutex);
            continue;
        }

        hbc::socket_packet* packet = m_packets.front();
        m_packets.pop_front();

        hbc::socket* socket = packet->sock;
        socket->write(packet->payload, packet->length);

        hbc::free_packet(packet);
    }
}

void* cmdr::sending_thread(void* arg)
{
    cmdr::packet_sender* sender = (cmdr::packet_sender*) arg;
    sender->run();
    return 0;
}
