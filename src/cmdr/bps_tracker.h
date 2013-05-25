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

#ifndef BPS_TRACKER_H
#define BPS_TRACKER_H

#include <sys/time.h>
#include <deque>

namespace cmdr
{

typedef enum
{
    none = 0,
    bps,
    kbps,
    mbps
} bps_type;

cmdr::bps_type get_bps_type(double bytes, double *type_out);
const char* b_type_str(cmdr::bps_type type);
const char* bps_type_str(cmdr::bps_type type);

class bps_tracker
{

public:

    bps_tracker();

    void start();
    void update(int bytes);
    bps_type get_bps(double* bps_out);

private:

    int m_bytes;
    std::deque<double> m_bps;
    struct timeval m_start;
    struct timeval m_end;
};

}

#endif
