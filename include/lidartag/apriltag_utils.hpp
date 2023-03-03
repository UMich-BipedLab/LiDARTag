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

#ifndef APRILTAG_UTILS_HPP
#define APRILTAG_UTILS_HPP

#include <lidartag/lidartag.hpp>

namespace BipedAprilLab{

    static inline int imax(int a, int b);


    /** if the bits in w were arranged in a d*d grid and that grid was
     * rotated, what would the new bits in w be?
     * The bits are organized like this (for d = 3):
     *
     *  8 7 6       2 5 8      0 1 2
     *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
     *  2 1 0       0 3 6      6 7 8
     **/
    uint64_t rotate90(uint64_t w, uint32_t d);

    void QuickDecodeAdd(BipedLab::QuickDecode_t *qd, uint64_t code, int id, int hamming);

    void QuickDecodeInit(BipedLab::GrizTagFamily_t *family, int maxhamming);
    void QuickDecodeCodeword(BipedLab::GrizTagFamily_t *tf, uint64_t rcode,
                             BipedLab::QuickDecodeEntry_t *entry);
} // namespace
#endif
