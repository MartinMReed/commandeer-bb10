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

#include <math.h>

#include "_math.h"

int hbc::num_digits(int num)
{
    return (num == 0) ? 1 : floor(log10(num) + 1);
}

void hbc::int_to_bytes(int i, unsigned char* b)
{
    b[3] = i & 0xFF;
    b[2] = (i >> 8) & 0xFF;
    b[1] = (i >> 16) & 0xFF;
    b[0] = (i >> 24) & 0xFF;
}

int hbc::bytes_to_int(unsigned char* b)
{
    int value = 0;
    for (int j = 0; j < 4; j++)
    {
        value = (value << 8) | (b[j] & 0xFF);
    }
    return value;
}
