/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "rtcm.h"

#include <cstring>

QGC_LOGGING_CATEGORY(RtcmParserLog, "RtcmParserLog")

RTCMParsing::RTCMParsing()
{
  reset();
}

RTCMParsing::~RTCMParsing() { delete[] _buffer; }

void RTCMParsing::reset() {
  if (_buffer && _buffer_len < RTCM_INITIAL_BUFFER_LENGTH) {
    delete[] _buffer;
    _buffer = nullptr;
    _buffer_len = 0;
  }

  if (!_buffer) {
    _buffer = new uint8_t[RTCM_INITIAL_BUFFER_LENGTH];
    _buffer_len = RTCM_INITIAL_BUFFER_LENGTH;
  }

  _pos = 0;
  _message_length = _buffer_len;
  _rtcm_version = 0;
}

bool RTCMParsing::addByte(uint8_t b)
{
	if (!_buffer) {
          qCDebug(RtcmParserLog, "RTCM: buffer allocation failed");
          return false;
	}

        if (_pos == 0) {
          if (b == RTCM2_PREAMBLE) {
            qCDebug(RtcmParserLog, "RTCM 2 detected");
            _rtcm_version = 2;
          } else if (b == RTCM3_PREAMBLE) {
            qCDebug(RtcmParserLog, "RTCM 3 detected");
            _rtcm_version = 3;
          } else {
            qCDebug(RtcmParserLog, "RTCM: unknown preamble 0x%02x", b);
            _rtcm_version = 0;
          }
        }

        if (_rtcm_version == 0) {
//          qCCritical(RtcmParserLog, "RTCM 0: unknown preamble 0x%02x", _buffer[0]);
          return false;
        }

        if (_rtcm_version == 2) {
          qCWarning(RtcmParserLog) << "RTCM 2 not supported";
          return false;
        }

        if (_pos >= _buffer_len) {
          qCDebug(RtcmParserLog, "RTCM: buffer overflow");
          return false;
        }
        _buffer[_pos++] = b;

        if (_rtcm_version == 3 && _pos == 3) {
          
		_message_length = (((uint16_t)_buffer[1] & 3) << 8) | (_buffer[2]);
                qCDebug(RtcmParserLog, "RTCM: message length %d", _message_length);

        if (_message_length + 6 > _buffer_len) {
			uint16_t new_buffer_len = _message_length + 6;
			uint8_t *new_buffer = new uint8_t[new_buffer_len];

			memcpy(new_buffer, _buffer, 3);
			delete[](_buffer);
			_buffer = new_buffer;
			_buffer_len = new_buffer_len;
		}
	}

	return _message_length + 6 == _pos;
}
