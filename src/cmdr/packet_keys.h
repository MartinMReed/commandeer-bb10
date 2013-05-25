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

#ifndef CMDR_SESSION_KEYS_H
#define CMDR_SESSION_KEYS_H

namespace cmdr
{

typedef enum
{
    cmdr_packet_error = 0,
    cmdr_mapping,
    cmdr_mapping_r,
    cmdr_connect_to,
    cmdr_connect_to_r,
    cmdr_partner_disconnected,
    cmdr_partner_pipe,
    cmdr_frame_index,
    cmdr_packet_ack,
    cmdr_packet_nak
} packet_key;

typedef enum
{
    unknown = 0,
    command,
    data
} connection_type;

typedef enum
{
    failed = 0,
    waiting_for_partner,
    connected
} connect_to_pin_response;

}

#endif
