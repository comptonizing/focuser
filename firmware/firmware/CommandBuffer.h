/*
 *  This file is part of the Pollux Rotator software
 *
 *  Created by Philipp Weber
 *  Copyright (c) 2023 Philipp Weber
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */



#pragma once

#define CMDBUFF_SIZE 32

#include <Arduino.h>
#include <util/crc16.h>

#include "Motor.h"

class CommandBuffer {
  public:
    static CommandBuffer &i();
    void clear();
    const char *get();
    const char *getCommand();
    bool add(char c);
    bool verifyChecksum();
  private:
    ~CommandBuffer();
    CommandBuffer();
    CommandBuffer(const CommandBuffer&);
    CommandBuffer & operator=(const CommandBuffer&);
    char m_buff[CMDBUFF_SIZE];
    char *pos = m_buff;
};
