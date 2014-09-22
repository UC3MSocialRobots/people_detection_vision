/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include "stereo_functions.h"

int main(int argc, char **argv) {
  image<uchar> *img1, *img2, *out, *edges;

  if (argc != 4) {
    std::cerr << "usage: " << argv[0] << " left(pgm) right(pgm) out(pgm)\n";
    exit(1);
  }

  // load input
  img1 = loadPGM(argv[1]);
  img2 = loadPGM(argv[2]);

  // compute disparities
  out = stereo_ms(img1, img2);

  // save output
  savePGM(out, argv[3]);
  
  delete img1;
  delete img2;
  delete out;
  return 0;
}
