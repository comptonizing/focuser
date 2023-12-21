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



#include "CommandBuffer.h"

CommandBuffer &CommandBuffer::i() {
  static CommandBuffer theInstance;
  return theInstance;
}

CommandBuffer::CommandBuffer() {
  clear();
}

CommandBuffer::~CommandBuffer() {
}

void CommandBuffer::clear() {
  m_buff[0] = '\0';
  pos = m_buff;
}

const char *CommandBuffer::get() {
  return m_buff;
}

bool CommandBuffer::add(char c) {
  if ( (pos - m_buff + 1) > CMDBUFF_SIZE ) {
    return false;
  }
  *pos++ = c;
  *pos = '\0';
  return true;
}

bool CommandBuffer::verifyChecksum() {
  char *start = m_buff + 1;
  int len = pos - m_buff - 5;
  Motor::i().update();
  uint16_t crcRef;
  memcpy(&crcRef, start + len + 1, 2);
  Motor::i().update();
  uint16_t crc = 0;
  for (int ii=0; ii<len; ii++) {
    Motor::i().update();
    crc = _crc16_update(crc, start[ii]);
  }
  char *crcChar = (char *) &crc;
  if ( crcChar[0] == '$' ) {
    crcChar[0] = '1';
  }
  if ( crcChar[1] == '$' ) {
    crcChar[1] = '1';
  }
  return crc == crcRef;
}

const char *CommandBuffer::getCommand() {
  return m_buff + 1;
}
