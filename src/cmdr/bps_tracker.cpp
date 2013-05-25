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

#include "bps_tracker.h"

cmdr::bps_tracker::bps_tracker()
{
    m_bytes = 0;
}

void cmdr::bps_tracker::start()
{
    gettimeofday(&m_start, NULL);
}

void cmdr::bps_tracker::update(int bytes)
{
    m_bytes += bytes;
    gettimeofday(&m_end, NULL);

    long seconds = m_end.tv_sec - m_start.tv_sec;
    if (seconds < 1) return;

    m_bps.push_back(m_bytes / (double) seconds);
    m_bytes = 0;
    m_start = m_end;

    if (m_bps.size() == 10)
    {
        m_bps.pop_front();
    }
}

cmdr::bps_type cmdr::bps_tracker::get_bps(double* bps_out)
{
    if (m_bps.empty()) return none;

    double n_bps = 0;
    for (int i = 0; i < (int) m_bps.size(); i++)
    {
        n_bps += m_bps[i];
    }

    n_bps /= (double) m_bps.size();
    *bps_out = n_bps;

    if (n_bps < 1024) return bps;
    else if (n_bps < 1024 * 1024)
    {
        *bps_out /= (double) 1024;
        return kbps;
    }
    else
    {
        *bps_out /= (double) (1024 * 1024);
        return mbps;
    }
}

cmdr::bps_type cmdr::get_bps_type(double bytes, double *type_out)
{
    if (bytes < 1024) return bps;
    else if (bytes < 1024 * 1024)
    {
        *type_out /= (double) 1024;
        return kbps;
    }
    else
    {
        *type_out /= (double) (1024 * 1024);
        return mbps;
    }
}

const char* cmdr::b_type_str(cmdr::bps_type type)
{
    switch (type)
    {
        case bps:
            return "b";
        case kbps:
            return "kb";
        case mbps:
            return "mb";
        default:
            return "?";
    }
}

const char* cmdr::bps_type_str(cmdr::bps_type type)
{
    switch (type)
    {
        case bps:
            return "b/s";
        case kbps:
            return "kb/s";
        case mbps:
            return "mb/s";
        default:
            return "?";
    }
}
