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

/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed in the APRIL Robotics Lab under the
 direction of Edwin Olson, ebolson@umich.edu. This software may be
 available under alternative licensing terms; contact the address above.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are
 those
 of the authors and should not be interpreted as representing official
 policies,
 either expressed or implied, of the Regents of The University of
 Michigan.
 */

#include <lidartag/apriltag_utils.hpp>

// Adapted/modified from apriltags code
namespace BipedAprilLab
{
static inline int imax(int a, int b) { return (a > b) ? a : b; }

/** if the bits in w were arranged in a d*d grid and that grid was
	 * rotated, what would the new bits in w be?
	 * The bits are organized like this (for d = 3):
	 *
	 *  8 7 6       2 5 8      0 1 2
	 *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
	 *  2 1 0       0 3 6      6 7 8
	 **/

uint64_t rotate90(uint64_t w, uint32_t d)
{
  uint64_t wr = 0;

  for (int32_t r = d - 1; r >= 0; r--) {
    for (int32_t c = 0; c < d; c++) {
      int32_t b = r + d * c;

      wr = wr << 1;

      if ((w & (((uint64_t)1) << b)) != 0) wr |= 1;
    }
  }

  return wr;
}

void QuickDecodeAdd(BipedLab::QuickDecode_t * qd, uint64_t code, int id, int hamming)
{
  uint32_t bucket = code % qd->nentries;

  while (qd->entries[bucket].rcode != UINT64_MAX) {
    bucket = (bucket + 1) % qd->nentries;
  }

  qd->entries[bucket].rcode = code;
  qd->entries[bucket].id = id;
  qd->entries[bucket].hamming = hamming;
}

void QuickDecodeInit(BipedLab::GrizTagFamily_t * family, int maxhamming)
{
  assert(family->impl == NULL);
  assert(family->ncodes < 65535);

  printf("Requesting memory\n");
  BipedLab::QuickDecode_t * qd = new BipedLab::QuickDecode_t;

  int capacity = family->ncodes;

  int nbits = family->d * family->d;

  if (maxhamming >= 1) capacity += family->ncodes * nbits;

  if (maxhamming >= 2) capacity += family->ncodes * nbits * (nbits - 1);

  if (maxhamming >= 3) capacity += family->ncodes * nbits * (nbits - 1) * (nbits - 2);

  if (maxhamming >= 4) capacity += family->ncodes * nbits * (nbits - 1) * (nbits - 2) * (nbits - 3);

  if (maxhamming >= 5)
    capacity += family->ncodes * nbits * (nbits - 1) * (nbits - 2) * (nbits - 3) * (nbits - 4);

  if (maxhamming >= 6)
    capacity +=
      family->ncodes * nbits * (nbits - 1) * (nbits - 2) * (nbits - 3) * (nbits - 4) * (nbits - 5);

  if (maxhamming >= 7)
    capacity += family->ncodes * nbits * (nbits - 1) * (nbits - 2) * (nbits - 3) * (nbits - 4) *
                (nbits - 5) * (nbits - 6);

  qd->nentries = capacity * 3;

  qd->entries = new BipedLab::QuickDecodeEntry_t[qd->nentries];
  if (qd->entries == NULL) {
    printf("apriltag_utils.c: failed to allocate hamming decode table. Reduce max hamming size.\n");
    exit(-1);
  }

  printf("Adding codes..\n");
  for (int i = 0; i < qd->nentries; i++) qd->entries[i].rcode = UINT64_MAX;

  for (int i = 0; i < family->ncodes; i++) {
    uint64_t code = family->codes[i];

    // add exact code (hamming = 0)
    QuickDecodeAdd(qd, code, i, 0);

    if (maxhamming >= 1) {
      // add hamming 1
      for (int j = 0; j < nbits; j++) QuickDecodeAdd(qd, code ^ (1L << j), i, 1);
    }

    if (maxhamming >= 2) {
      // add hamming 2
      for (int j = 0; j < nbits; j++)
        for (int k = 0; k < j; k++) QuickDecodeAdd(qd, code ^ (1L << j) ^ (1L << k), i, 2);
    }

    if (maxhamming >= 3) {
      // add hamming 3
      for (int j = 0; j < nbits; j++)
        for (int k = 0; k < j; k++)
          for (int m = 0; m < k; m++)
            QuickDecodeAdd(qd, code ^ (1L << j) ^ (1L << k) ^ (1L << m), i, 3);
    }

    if (maxhamming >= 4) {
      // add hamming 3
      for (int j = 0; j < nbits; j++)
        for (int k = 0; k < j; k++)
          for (int m = 0; m < k; m++)
            for (int n = 0; n < m; n++)
              QuickDecodeAdd(qd, code ^ (1L << j) ^ (1L << k) ^ (1L << m) ^ (1L << n), i, 4);
    }

    if (maxhamming >= 5) {
      // add hamming 3
      for (int j = 0; j < nbits; j++)
        for (int k = 0; k < j; k++)
          for (int m = 0; m < k; m++)
            for (int n = 0; n < m; n++)
              for (int o = 0; o < n; o++)
                QuickDecodeAdd(
                  qd, code ^ (1L << j) ^ (1L << k) ^ (1L << m) ^ (1L << n) ^ (1L << o), i, 5);
    }

    if (maxhamming >= 6) {
      // add hamming 3
      for (int j = 0; j < nbits; j++)
        for (int k = 0; k < j; k++)
          for (int m = 0; m < k; m++)
            for (int n = 0; n < m; n++)
              for (int o = 0; o < n; o++)
                for (int p = 0; p < o; p++)
                  QuickDecodeAdd(
                    qd,
                    code ^ (1L << j) ^ (1L << k) ^ (1L << m) ^ (1L << n) ^ (1L << o) ^ (1L << p), i,
                    6);
    }

    if (maxhamming >= 7) {
      // add hamming 3
      for (int j = 0; j < nbits; j++)
        for (int k = 0; k < j; k++)
          for (int m = 0; m < k; m++)
            for (int n = 0; n < m; n++)
              for (int o = 0; o < n; o++)
                for (int p = 0; p < o; p++)
                  for (int q = 0; q < p; q++)
                    QuickDecodeAdd(
                      qd,
                      code ^ (1L << j) ^ (1L << k) ^ (1L << m) ^ (1L << n) ^ (1L << o) ^ (1L << p) ^
                        (1L << q),
                      i, 7);
    }

    if (maxhamming > 7) {
      printf("apriltag_utils.c: maxhamming beyond 7 not supported\n");
    }
  }

  // printf("done adding..\n");
  family->impl = qd;

  if (1) {
    int longest_run = 0;
    int run = 0;
    int run_sum = 0;
    int run_count = 0;

    // This accounting code doesn't check the last possible run that
    // occurs at the wrap-around. That's pretty insignificant.
    for (int i = 0; i < qd->nentries; i++) {
      if (qd->entries[i].rcode == UINT64_MAX) {
        if (run > 0) {
          run_sum += run;
          run_count++;
        }
        run = 0;
      } else {
        run++;
        longest_run = imax(longest_run, run);
      }
    }

    printf(
      "quick decode: longest run: %d, average run %.3f\n", longest_run, 1.0 * run_sum / run_count);
  }
}

void QuickDecodeCodeword(
  BipedLab::GrizTagFamily_t * tf, uint64_t rcode, BipedLab::QuickDecodeEntry_t * entry)
{
  BipedLab::QuickDecode_t * qd = (BipedLab::QuickDecode_t *)tf->impl;

  for (int ridx = 0; ridx < 4; ridx++) {
    for (int bucket = rcode % qd->nentries; qd->entries[bucket].rcode != UINT64_MAX;
         bucket = (bucket + 1) % qd->nentries) {
      // std::cout << "bucket: " << bucket << std::endl;

      if (qd->entries[bucket].rcode == rcode) {
        *entry = qd->entries[bucket];
        entry->rotation = ridx;
        //printf("rotation: %i\n", entry->rotation);
        return;
      }
    }

    rcode = rotate90(rcode, tf->d);
  }

  entry->rcode = 0;
  entry->id = 9999;
  entry->hamming = 255;
  entry->rotation = 0;
}
}  // namespace BipedAprilLab
