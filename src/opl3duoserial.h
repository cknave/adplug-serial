/*
 * Adplug - Replayer for many OPL2/OPL3 audio file formats.
 * Copyright (C) 1999 - 2005 Simon Peter, <dn.tlp@gmx.net>, et al.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * opl3duoserial.h - Serial interface to an OPL3 Duo!, by Kevin Vance <kvance@kvance.com>
 */

#ifndef H_ADPLUG_OPL3DUOSERIAL
#define H_ADPLUG_OPL3DUOSERIAL

#include <ctime>
#include "opl.h"

enum ResetMode {
    ReturnBeforeComplete,
    WaitUntilComplete,
};

class CSerialDuoOpl: public Copl
{
private:
    const char *device;
    const int baud;
    int fd;
    struct timespec lastWrite;

public:
    CSerialDuoOpl(const char *device, int baud);
    virtual ~CSerialDuoOpl();

    void write(int reg, int val);

    void init();

    void reset(enum ResetMode mode = WaitUntilComplete);

private:
    static struct timespec diffTime(const struct timespec &t1, struct timespec t2);
};

#endif
