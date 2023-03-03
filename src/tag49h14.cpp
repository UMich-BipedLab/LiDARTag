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

#include <lidartag/tag49h14.hpp>
#include <stdlib.h>

BipedLab::GrizTagFamily_t * tag49h14_create()
{
  BipedLab::GrizTagFamily_t * tf =
    (BipedLab::GrizTagFamily_t *)calloc(1, sizeof(BipedLab::GrizTagFamily_t));
  tf->black_border = 1;
  tf->d = 7;
  tf->h = 14;
  tf->ncodes = 50;
  tf->codes = (uint64_t *)calloc(50, sizeof(uint64_t));
  tf->codes[1] = 0x00000fc0c319c1ffUL;
  tf->codes[2] = 0x000033e0cc7c0c63UL;
  tf->codes[3] = 0x0000703e339cce0fUL;
  tf->codes[4] = 0x0001cf38c31f3198UL;
  tf->codes[5] = 0x0001f019cf1cc663UL;
  tf->codes[6] = 0x00018f99f19cf1f8UL;
  tf->codes[7] = 0x000060ff81983e0cUL;
  tf->codes[8] = 0x0001cf18ce03387fUL;
  tf->codes[9] = 0x00018fe07e00c673UL;
  tf->codes[10] = 0x0001cf060018ce73UL;
  tf->codes[11] = 0x0000633f3c60c673UL;
  tf->codes[12] = 0x0001e386307f0c73UL;
  tf->codes[13] = 0x00003039ce67fc0fUL;
  tf->codes[14] = 0x00003f98c318fe78UL;
  tf->codes[15] = 0x00007f9f807f31f0UL;
  tf->codes[16] = 0x00001f1f01fcfe0cUL;
  tf->codes[17] = 0x0001e3007e603073UL;
  tf->codes[18] = 0x0001e07f30063e0fUL;
  tf->codes[19] = 0x0000701fc0e70c73UL;
  tf->codes[20] = 0x00019ce63f9ccc07UL;
  tf->codes[21] = 0x00019cc7ce660e0cUL;
  tf->codes[22] = 0x00001f98cc673e0cUL;
  tf->codes[23] = 0x000070673c67f19cUL;
  tf->codes[24] = 0x00000fe07e06318fUL;
  tf->codes[25] = 0x0001cce0f18601f0UL;
  tf->codes[26] = 0x0001cf003f01f1f0UL;
  tf->codes[27] = 0x00018fc73318398cUL;
  tf->codes[28] = 0x00019c7f0187f998UL;
  tf->codes[29] = 0x0001e319cf07c38cUL;
  tf->codes[30] = 0x00001c667fe1f80fUL;
  tf->codes[31] = 0x0001cc6181fe3c1cUL;
  tf->codes[32] = 0x00003fc18181fe0fUL;
  tf->codes[33] = 0x00003f0671fff18cUL;
  tf->codes[34] = 0x0001e39981e3f198UL;
  tf->codes[35] = 0x00003c6000e70c07UL;
  tf->codes[36] = 0x00007fe70cfc319cUL;
  tf->codes[37] = 0x0001e31e7001fc1cUL;
  tf->codes[38] = 0x00007f383ce3fc63UL;
  tf->codes[39] = 0x00001c660e063e67UL;
  tf->codes[40] = 0x00018398ce1803e3UL;
  tf->codes[41] = 0x00003f39f19fcc07UL;
  tf->codes[42] = 0x0001cce18e030187UL;
  tf->codes[43] = 0x000033c0f19c3e18UL;
  tf->codes[44] = 0x00007338c1fe001cUL;
  tf->codes[45] = 0x00007f8633033198UL;
  tf->codes[46] = 0x0001cf39fe7e0607UL;
  tf->codes[47] = 0x0001f3c67f1cce60UL;
  tf->codes[48] = 0x0001e3380f99f803UL;
  tf->codes[49] = 0x00018c6181f8c3f8UL;
  tf->codes[50] = 0x00001fe1f1f8ce70UL;
  return tf;
}

void tag49h14_destroy(BipedLab::GrizTagFamily_t * tf)
{
  free(tf->name);
  free(tf->codes);
  free(tf);
}
