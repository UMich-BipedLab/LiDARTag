/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.brucerobot.com/
 */

#ifndef ULTRA_PUCK_HPP
#define ULTRA_PuCK_HPP

namespace BipedLab
{
namespace UltraPuckV2
{
// Number of beams in Velodyne Ultra Puck 2.0
constexpr int beams = 32;

/* el holds the elevation angle information for each ring in the UltraPuckV2.
 * The number to the right of the elevation angle is the hardware laser id
 * which can be found in the VLP-32C manual.
 */
constexpr float el[] = {
    -25.0,   // 0
    -15.639, // 3
    -11.310, // 4
    -8.843,  // 7
    -7.254,  // 8
    -6.148,  // 11
    -5.333,  // 12
    -4.667,  // 16
    -4.0,    // 15
    -3.667,  // 19
    -3.333,  // 20
    -3.0,    // 24
    -2.667,  // 23
    -2.333,  // 27
    -2.0,    // 28
    -1.667,  // 2
    -1.333,  // 31
    -1.0,    // 1
    -0.667,  // 6
    -0.333,  // 10
    0.0,     // 5
    0.333,   // 9
    0.667,   // 14
    1.0,     // 18
    1.333,   // 13
    1.667,   // 17
    2.333,   // 22
    3.333,   // 21
    4.667,   // 26
    7.0,     // 25
    10.333,  // 30
    15.0     // 29
};

constexpr float AZ_RESOLUTION_300RPM = 0.1;
constexpr float AZ_RESOLUTION_600RPM = 0.2;
constexpr float AZ_RESOLUTION_900RPM = 0.3;
constexpr float AZ_RESOLUTION_1200RPM = 0.4;

struct EL_TABLE
{
   constexpr EL_TABLE() : values()
   {
      for (auto i = 0; i < 32; ++i) {
         values[i] = tan(el[i]*M_PI/180);
      }
   }

   int values[32];
};

constexpr EL_TABLE EL_TAN = EL_TABLE();

} // namespace UltraPuckV2
} // namespace BipedLab

#endif
