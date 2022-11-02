/*
  FX_2Dfcn.cpp contains all 2D utility functions
  
  LICENSE
  The MIT License (MIT)
  Copyright (c) 2022  Blaz Kristan (https://blaz.at/home)
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

  Parts of the code adapted from WLED Sound Reactive
*/
#include "wled.h"
#include "FX.h"
#include "palettes.h"

// setUpMatrix() - constructs ledmap array from matrix of panels with WxH pixels
// this converts physical (possibly irregular) LED arrangement into well defined
// array of logical pixels: fist entry corresponds to left-topmost logical pixel
// followed by horizontal pixels, when matrixWidth logical pixels are added they
// are followed by next row (down) of matrixWidth pixels (and so forth)
// note: matrix may be comprised of multiple panels each with different orientation
// but ledmap takes care of that. ledmap is constructed upon initialization
// so matrix should disable regular ledmap processing
void WS2812FX::setUpMatrix() {
#ifndef WLED_DISABLE_2D
  // erase old ledmap, just in case.
  if (customMappingTable != nullptr) delete[] customMappingTable;
  customMappingTable = nullptr;
  customMappingSize = 0;

  if (isMatrix) {
    matrixWidth  = hPanels * panelW;
    matrixHeight = vPanels * panelH;

    // safety check
    if (matrixWidth * matrixHeight > MAX_LEDS) {
      matrixWidth = _length;
      matrixHeight = 1;
      isMatrix = false;
      return;
    }

    customMappingSize  = matrixWidth * matrixHeight;
    customMappingTable = new uint16_t[customMappingSize];

    if (customMappingTable != nullptr) {
      uint16_t startL; // index in custom mapping array (logical strip)
      uint16_t startP; // position of 1st pixel of panel on (virtual) strip
      uint16_t x, y, offset;
      uint8_t h = matrix.vertical ? vPanels : hPanels;
      uint8_t v = matrix.vertical ? hPanels : vPanels;

      for (uint8_t j=0, p=0; j<v; j++) {
        for (uint8_t i=0; i<h; i++, p++) {
          y = (matrix.vertical ? matrix.rightStart : matrix.bottomStart) ? v - j - 1 : j;
          x = (matrix.vertical ? matrix.bottomStart : matrix.rightStart) ? h - i - 1 : i;
          x = matrix.serpentine && j%2 ? h - x - 1 : x;

          startL = (matrix.vertical ? y : x) * panelW + (matrix.vertical ? x : y) * matrixWidth * panelH; // logical index (top-left corner)
          startP = p * panelW * panelH; // physical index (top-left corner)

          uint8_t H = panel[h*j + i].vertical ? panelW : panelH;
          uint8_t W = panel[h*j + i].vertical ? panelH : panelW;
          for (uint16_t l=0, q=0; l<H; l++) {
            for (uint16_t k=0; k<W; k++, q++) {
              y = (panel[h*j + i].vertical ? panel[h*j + i].rightStart : panel[h*j + i].bottomStart) ? H - l - 1 : l;
              x = (panel[h*j + i].vertical ? panel[h*j + i].bottomStart : panel[h*j + i].rightStart) ? W - k - 1 : k;
              x = (panel[h*j + i].serpentine && l%2) ? (W - x - 1) : x;
              offset = (panel[h*j + i].vertical ? y : x) + (panel[h*j + i].vertical ? x : y) * matrixWidth;
              customMappingTable[startL + offset] = startP + q;
            }
          }
        }
      }
      #ifdef WLED_DEBUG
      DEBUG_PRINT(F("Matrix ledmap:"));
      for (uint16_t i=0; i<customMappingSize; i++) {
        if (!(i%matrixWidth)) DEBUG_PRINTLN();
        DEBUG_PRINTF("%4d,", customMappingTable[i]);
      }
      DEBUG_PRINTLN();
      #endif
    } else {
      // memory allocation error
      matrixWidth = _length;
      matrixHeight = 1;
      isMatrix = false;
      return;
    }
  } else { 
    // not a matrix set up
    matrixWidth = _length;
    matrixHeight = 1;
  }
#endif
}

// absolute matrix version of setPixelColor()
void IRAM_ATTR WS2812FX::setPixelColorXY(int x, int y, uint32_t col)
{
#ifndef WLED_DISABLE_2D
  if (!isMatrix) return; // not a matrix set-up
  uint16_t index = y * matrixWidth + x;
#else
  uint16_t index = x;
#endif
  if (index >= _length) return;
  if (index < customMappingSize) index = customMappingTable[index];
  busses.setPixelColor(index, col);
}

// returns RGBW values of pixel
uint32_t WS2812FX::getPixelColorXY(uint16_t x, uint16_t y) {
#ifndef WLED_DISABLE_2D
  uint16_t index = (y * matrixWidth + x);
#else
  uint16_t index = x;
#endif
  if (index >= _length) return 0;
  if (index < customMappingSize) index = customMappingTable[index];
  return busses.getPixelColor(index);
}

///////////////////////////////////////////////////////////
// Segment:: routines
///////////////////////////////////////////////////////////

#ifndef WLED_DISABLE_2D

// XY(x,y) - gets pixel index within current segment (often used to reference leds[] array element)
uint16_t IRAM_ATTR Segment::XY(uint16_t x, uint16_t y) {
  uint16_t width  = virtualWidth();   // segment width in logical pixels
  uint16_t height = virtualHeight();  // segment height in logical pixels
  return (x%width) + (y%height) * width;
}

void IRAM_ATTR Segment::setPixelColorXY(int x, int y, uint32_t col)
{
  if (!strip.isMatrix) return; // not a matrix set-up

  if (leds) leds[XY(x,y)] = col;

  uint8_t _bri_t = currentBri(on ? opacity : 0);
  if (_bri_t < 255) {
    byte r = scale8(R(col), _bri_t);
    byte g = scale8(G(col), _bri_t);
    byte b = scale8(B(col), _bri_t);
    byte w = scale8(W(col), _bri_t);
    col = RGBW32(r, g, b, w);
  }

  if (reverse  ) x = virtualWidth()  - x - 1;
  if (reverse_y) y = virtualHeight() - y - 1;
  if (transpose) { uint16_t t = x; x = y; y = t; } // swap X & Y if segment transposed

  x *= groupLength(); // expand to physical pixels
  y *= groupLength(); // expand to physical pixels
  if (x >= width() || y >= height()) return;  // if pixel would fall out of segment just exit

  for (int j = 0; j < grouping; j++) {   // groupping vertically
    for (int g = 0; g < grouping; g++) { // groupping horizontally
      uint16_t xX = (x+g), yY = (y+j);
      if (xX >= width() || yY >= height()) continue; // we have reached one dimension's end

      strip.setPixelColorXY(start + xX, startY + yY, col);

      if (mirror) { //set the corresponding horizontally mirrored pixel
        if (transpose) strip.setPixelColorXY(start + xX, startY + height() - yY - 1, col);
        else           strip.setPixelColorXY(start + width() - xX - 1, startY + yY, col);
      }
      if (mirror_y) { //set the corresponding vertically mirrored pixel
        if (transpose) strip.setPixelColorXY(start + width() - xX - 1, startY + yY, col);
        else           strip.setPixelColorXY(start + xX, startY + height() - yY - 1, col);
      }
      if (mirror_y && mirror) { //set the corresponding vertically AND horizontally mirrored pixel
        strip.setPixelColorXY(width() - xX - 1, height() - yY - 1, col);
      }
    }
  }
}

// anti-aliased version of setPixelColorXY()
void Segment::setPixelColorXY(float x, float y, uint32_t col, bool aa)
{
  if (!strip.isMatrix) return; // not a matrix set-up
  if (x<0.0f || x>1.0f || y<0.0f || y>1.0f) return; // not normalized

  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();

  float fX = x * (cols-1);
  float fY = y * (rows-1);
  if (aa) {
    uint16_t xL = roundf(fX-0.49f);
    uint16_t xR = roundf(fX+0.49f);
    uint16_t yT = roundf(fY-0.49f);
    uint16_t yB = roundf(fY+0.49f);
    float    dL = fX - xL;
    float    dR = xR - fX;
    float    dT = fY - yT;
    float    dB = yB - fY;
    uint32_t cXLYT = getPixelColorXY(xL, yT);
    uint32_t cXRYT = getPixelColorXY(xR, yT);
    uint32_t cXLYB = getPixelColorXY(xL, yB);
    uint32_t cXRYB = getPixelColorXY(xR, yB);

    if (xL!=xR && yT!=yB) {
      setPixelColorXY(xL, yT, color_blend(col, cXLYT, uint8_t(sqrtf(dL*dT)*255.0f))); // blend TL pixel
      setPixelColorXY(xR, yT, color_blend(col, cXRYT, uint8_t(sqrtf(dR*dT)*255.0f))); // blend TR pixel
      setPixelColorXY(xL, yB, color_blend(col, cXLYB, uint8_t(sqrtf(dL*dB)*255.0f))); // blend BL pixel
      setPixelColorXY(xR, yB, color_blend(col, cXRYB, uint8_t(sqrtf(dR*dB)*255.0f))); // blend BR pixel
    } else if (xR!=xL && yT==yB) {
      setPixelColorXY(xR, yT, color_blend(col, cXLYT, uint8_t(dL*255.0f))); // blend L pixel
      setPixelColorXY(xR, yT, color_blend(col, cXRYT, uint8_t(dR*255.0f))); // blend R pixel
    } else if (xR==xL && yT!=yB) {
      setPixelColorXY(xR, yT, color_blend(col, cXLYT, uint8_t(dT*255.0f))); // blend T pixel
      setPixelColorXY(xL, yB, color_blend(col, cXLYB, uint8_t(dB*255.0f))); // blend B pixel
    } else {
      setPixelColorXY(xL, yT, col); // exact match (x & y land on a pixel)
    }
  } else {
    setPixelColorXY(uint16_t(roundf(fX)), uint16_t(roundf(fY)), col);
  }
}

// returns RGBW values of pixel
uint32_t Segment::getPixelColorXY(uint16_t x, uint16_t y) {
  int i = XY(x,y);
  if (leds) return RGBW32(leds[i].r, leds[i].g, leds[i].b, 0);
  if (reverse  ) x = virtualWidth()  - x - 1;
  if (reverse_y) y = virtualHeight() - y - 1;
  if (transpose) { uint16_t t = x; x = y; y = t; } // swap X & Y if segment transposed
  x *= groupLength(); // expand to physical pixels
  y *= groupLength(); // expand to physical pixels
  if (x >= width() || y >= height()) return 0;
  return strip.getPixelColorXY(start + x, startY + y);
}

// Blends the specified color with the existing pixel color.
void Segment::blendPixelColorXY(uint16_t x, uint16_t y, uint32_t color, uint8_t blend) {
  setPixelColorXY(x, y, color_blend(getPixelColorXY(x,y), color, blend));
}

// Adds the specified color with the existing pixel color perserving color balance.
void Segment::addPixelColorXY(int x, int y, uint32_t color) {
  setPixelColorXY(x, y, color_add(getPixelColorXY(x,y), color));
}

void Segment::fadePixelColorXY(uint16_t x, uint16_t y, uint8_t fade) {
  CRGB pix = CRGB(getPixelColorXY(x,y)).nscale8_video(fade);
  setPixelColor(x, y, pix);
}

// blurRow: perform a blur on a row of a rectangular matrix
void Segment::blurRow(uint16_t row, fract8 blur_amount) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();

  if (row >= rows) return;
  // blur one row
  uint8_t keep = 255 - blur_amount;
  uint8_t seep = blur_amount >> 1;
  CRGB carryover = CRGB::Black;
  for (uint16_t x = 0; x < cols; x++) {
    CRGB cur = getPixelColorXY(x, row);
    CRGB part = cur;
    part.nscale8(seep);
    cur.nscale8(keep);
    cur += carryover;
    if (x) {
      CRGB prev = CRGB(getPixelColorXY(x-1, row)) + part;
      setPixelColorXY(x-1, row, prev);
    }
    setPixelColorXY(x, row, cur);
    carryover = part;
  }
}

// blurCol: perform a blur on a column of a rectangular matrix
void Segment::blurCol(uint16_t col, fract8 blur_amount) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();

  if (col >= cols) return;
  // blur one column
  uint8_t keep = 255 - blur_amount;
  uint8_t seep = blur_amount >> 1;
  CRGB carryover = CRGB::Black;
  for (uint16_t i = 0; i < rows; i++) {
    CRGB cur = getPixelColorXY(col, i);
    CRGB part = cur;
    part.nscale8(seep);
    cur.nscale8(keep);
    cur += carryover;
    if (i) {
      CRGB prev = CRGB(getPixelColorXY(col, i-1)) + part;
      setPixelColorXY(col, i-1, prev);
    }
    setPixelColorXY(col, i, cur);
    carryover = part;
  }
}

// 1D Box blur (with added weight - blur_amount: [0=no blur, 255=max blur])
void Segment::box_blur(uint16_t i, bool vertical, fract8 blur_amount) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();
  const uint16_t dim1 = vertical ? rows : cols;
  const uint16_t dim2 = vertical ? cols : rows;
  if (i >= dim2) return;
  const float seep = blur_amount/255.f;
  const float keep = 3.f - 2.f*seep;
  // 1D box blur
  CRGB tmp[dim1];
  for (uint16_t j = 0; j < dim1; j++) {
    uint16_t x = vertical ? i : j;
    uint16_t y = vertical ? j : i;
    uint16_t xp = vertical ? x : x-1;
    uint16_t yp = vertical ? y-1 : y;
    uint16_t xn = vertical ? x : x+1;
    uint16_t yn = vertical ? y+1 : y;
    CRGB curr = getPixelColorXY(x,y);
    CRGB prev = (xp<0 || yp<0) ? CRGB::Black : getPixelColorXY(xp,yp);
    CRGB next = ((vertical && yn>=dim1) || (!vertical && xn>=dim1)) ? CRGB::Black : getPixelColorXY(xn,yn);
    uint16_t r, g, b;
    r = (curr.r*keep + (prev.r + next.r)*seep) / 3;
    g = (curr.g*keep + (prev.g + next.g)*seep) / 3;
    b = (curr.b*keep + (prev.b + next.b)*seep) / 3;
    tmp[j] = CRGB(r,g,b);
  }
  for (uint16_t j = 0; j < dim1; j++) {
    uint16_t x = vertical ? i : j;
    uint16_t y = vertical ? j : i;
    setPixelColorXY(x, y, tmp[j]);
  }
}

// blur1d: one-dimensional blur filter. Spreads light to 2 line neighbors.
// blur2d: two-dimensional blur filter. Spreads light to 8 XY neighbors.
//
//           0 = no spread at all
//          64 = moderate spreading
//         172 = maximum smooth, even spreading
//
//         173..255 = wider spreading, but increasing flicker
//
//         Total light is NOT entirely conserved, so many repeated
//         calls to 'blur' will also result in the light fading,
//         eventually all the way to black; this is by design so that
//         it can be used to (slowly) clear the LEDs to black.

void Segment::blur1d(fract8 blur_amount) {
  const uint16_t rows = virtualHeight();
  for (uint16_t y = 0; y < rows; y++) blurRow(y, blur_amount);
}

void Segment::moveX(int8_t delta) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();
  if (!delta) return;
  if (delta > 0) {
    for (uint8_t y = 0; y < rows; y++) for (uint8_t x = 0; x < cols-1; x++) {
      if (x + delta >= cols) break;
      setPixelColorXY(x, y, getPixelColorXY((x + delta)%cols, y));
    }
  } else {
    for (uint8_t y = 0; y < rows; y++) for (int16_t x = cols-1; x >= 0; x--) {
      if (x + delta < 0) break;
      setPixelColorXY(x, y, getPixelColorXY(x + delta, y));
    }
  }
}

void Segment::moveY(int8_t delta) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();
  if (!delta) return;
  if (delta > 0) {
    for (uint8_t x = 0; x < cols; x++) for (uint8_t y = 0; y < rows-1; y++) {
      if (y + delta >= rows) break;
      setPixelColorXY(x, y, getPixelColorXY(x, (y + delta)));
    }
  } else {
    for (uint8_t x = 0; x < cols; x++) for (int16_t y = rows-1; y >= 0; y--) {
      if (y + delta < 0) break;
      setPixelColorXY(x, y, getPixelColorXY(x, y + delta));
    }
  }
}

// move() - move all pixels in desired direction delta number of pixels
// @param dir direction: 0=left, 1=left-up, 2=up, 3=right-up, 4=right, 5=right-down, 6=down, 7=left-down
// @param delta number of pixels to move
void Segment::move(uint8_t dir, uint8_t delta) {
  if (delta==0) return;
  switch (dir) {
    case 0: moveX( delta);                break;
    case 1: moveX( delta); moveY( delta); break;
    case 2:                moveY( delta); break;
    case 3: moveX(-delta); moveY( delta); break;
    case 4: moveX(-delta);                break;
    case 5: moveX(-delta); moveY(-delta); break;
    case 6:                moveY(-delta); break;
    case 7: moveX( delta); moveY(-delta); break;
  }
}

// by stepko, taken from https://editor.soulmatelights.com/gallery/573-blobs
void Segment::fill_circle(uint16_t cx, uint16_t cy, uint8_t radius, CRGB col) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();
  for (int16_t y = -radius; y <= radius; y++) {
    for (int16_t x = -radius; x <= radius; x++) {
      if (x * x + y * y <= radius * radius &&
          int16_t(cx)+x>=0 && int16_t(cy)+y>=0 &&
          int16_t(cx)+x<cols && int16_t(cy)+y<rows)
        addPixelColorXY(cx + x, cy + y, col);
    }
  }
}

void Segment::nscale8(uint8_t scale) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();
  for(uint16_t y = 0; y < rows; y++) for (uint16_t x = 0; x < cols; x++) {
    if (leds) leds[XY(x,y)].nscale8(scale);
    else setPixelColorXY(x, y, CRGB(getPixelColorXY(x, y)).nscale8(scale));
  }
}

//line function
void Segment::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t c) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();
  if (x0 >= cols || x1 >= cols || y0 >= rows || y1 >= rows) return;
  const int16_t dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  const int16_t dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
  int16_t err = (dx>dy ? dx : -dy)/2, e2;
  for (;;) {
    setPixelColorXY(x0,y0,c);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}

// font curtesy of https://github.com/idispatch/raster-fonts
static const unsigned char console_font_6x8[] PROGMEM = {

    /*
     * code=0, hex=0x00, ascii="^@"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=1, hex=0x01, ascii="^A"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x6C,  /* 011011 */
    0x44,  /* 010001 */
    0x54,  /* 010101 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=2, hex=0x02, ascii="^B"
     */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x54,  /* 010101 */
    0x7C,  /* 011111 */
    0x44,  /* 010001 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=3, hex=0x03, ascii="^C"
     */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x7C,  /* 011111 */
    0x7C,  /* 011111 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=4, hex=0x04, ascii="^D"
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=5, hex=0x05, ascii="^E"
     */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x7C,  /* 011111 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=6, hex=0x06, ascii="^F"
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=7, hex=0x07, ascii="^G"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=8, hex=0x08, ascii="^H"
     */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xCC,  /* 110011 */
    0xCC,  /* 110011 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */

    /*
     * code=9, hex=0x09, ascii="^I"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=10, hex=0x0A, ascii="^J"
     */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0x84,  /* 100001 */
    0xB4,  /* 101101 */
    0xB4,  /* 101101 */
    0x84,  /* 100001 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */

    /*
     * code=11, hex=0x0B, ascii="^K"
     */
    0x00,  /* 000000 */
    0x1C,  /* 000111 */
    0x0C,  /* 000011 */
    0x34,  /* 001101 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=12, hex=0x0C, ascii="^L"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=13, hex=0x0D, ascii="^M"
     */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x14,  /* 000101 */
    0x10,  /* 000100 */
    0x30,  /* 001100 */
    0x70,  /* 011100 */
    0x60,  /* 011000 */
    0x00,  /* 000000 */

    /*
     * code=14, hex=0x0E, ascii="^N"
     */
    0x0C,  /* 000011 */
    0x34,  /* 001101 */
    0x2C,  /* 001011 */
    0x34,  /* 001101 */
    0x2C,  /* 001011 */
    0x6C,  /* 011011 */
    0x60,  /* 011000 */
    0x00,  /* 000000 */

    /*
     * code=15, hex=0x0F, ascii="^O"
     */
    0x00,  /* 000000 */
    0x54,  /* 010101 */
    0x38,  /* 001110 */
    0x6C,  /* 011011 */
    0x38,  /* 001110 */
    0x54,  /* 010101 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=16, hex=0x10, ascii="^P"
     */
    0x20,  /* 001000 */
    0x30,  /* 001100 */
    0x38,  /* 001110 */
    0x3C,  /* 001111 */
    0x38,  /* 001110 */
    0x30,  /* 001100 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */

    /*
     * code=17, hex=0x11, ascii="^Q"
     */
    0x08,  /* 000010 */
    0x18,  /* 000110 */
    0x38,  /* 001110 */
    0x78,  /* 011110 */
    0x38,  /* 001110 */
    0x18,  /* 000110 */
    0x08,  /* 000010 */
    0x00,  /* 000000 */

    /*
     * code=18, hex=0x12, ascii="^R"
     */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=19, hex=0x13, ascii="^S"
     */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=20, hex=0x14, ascii="^T"
     */
    0x3C,  /* 001111 */
    0x54,  /* 010101 */
    0x54,  /* 010101 */
    0x34,  /* 001101 */
    0x14,  /* 000101 */
    0x14,  /* 000101 */
    0x14,  /* 000101 */
    0x00,  /* 000000 */

    /*
     * code=21, hex=0x15, ascii="^U"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x30,  /* 001100 */
    0x28,  /* 001010 */
    0x18,  /* 000110 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=22, hex=0x16, ascii="^V"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=23, hex=0x17, ascii="^W"
     */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */

    /*
     * code=24, hex=0x18, ascii="^X"
     */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=25, hex=0x19, ascii="^Y"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=26, hex=0x1A, ascii="^Z"
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x7C,  /* 011111 */
    0x18,  /* 000110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=27, hex=0x1B, ascii="^["
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x30,  /* 001100 */
    0x7C,  /* 011111 */
    0x30,  /* 001100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=28, hex=0x1C, ascii="^\"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */

    /*
     * code=29, hex=0x1D, ascii="^]"
     */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x7C,  /* 011111 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=30, hex=0x1E, ascii="^^"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=31, hex=0x1F, ascii="^_"
     */
    0x7C,  /* 011111 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=32, hex=0x20, ascii=" "
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=33, hex=0x21, ascii="!"
     */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=34, hex=0x22, ascii="""
     */
    0x6C,  /* 011011 */
    0x6C,  /* 011011 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=35, hex=0x23, ascii="#"
     */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x7C,  /* 011111 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x7C,  /* 011111 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=36, hex=0x24, ascii="$"
     */
    0x20,  /* 001000 */
    0x38,  /* 001110 */
    0x40,  /* 010000 */
    0x30,  /* 001100 */
    0x08,  /* 000010 */
    0x70,  /* 011100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=37, hex=0x25, ascii="%"
     */
    0x64,  /* 011001 */
    0x64,  /* 011001 */
    0x08,  /* 000010 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x4C,  /* 010011 */
    0x4C,  /* 010011 */
    0x00,  /* 000000 */

    /*
     * code=38, hex=0x26, ascii="&"
     */
    0x20,  /* 001000 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x20,  /* 001000 */
    0x54,  /* 010101 */
    0x48,  /* 010010 */
    0x34,  /* 001101 */
    0x00,  /* 000000 */

    /*
     * code=39, hex=0x27, ascii="'"
     */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=40, hex=0x28, ascii="("
     */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=41, hex=0x29, ascii=")"
     */
    0x20,  /* 001000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */

    /*
     * code=42, hex=0x2A, ascii="*"
     */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x38,  /* 001110 */
    0x7C,  /* 011111 */
    0x38,  /* 001110 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=43, hex=0x2B, ascii="+"
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=44, hex=0x2C, ascii=","
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x20,  /* 001000 */

    /*
     * code=45, hex=0x2D, ascii="-"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=46, hex=0x2E, ascii="."
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=47, hex=0x2F, ascii="/"
     */
    0x00,  /* 000000 */
    0x04,  /* 000001 */
    0x08,  /* 000010 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x40,  /* 010000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=48, hex=0x30, ascii="0"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x4C,  /* 010011 */
    0x54,  /* 010101 */
    0x64,  /* 011001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=49, hex=0x31, ascii="1"
     */
    0x10,  /* 000100 */
    0x30,  /* 001100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=50, hex=0x32, ascii="2"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x04,  /* 000001 */
    0x18,  /* 000110 */
    0x20,  /* 001000 */
    0x40,  /* 010000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */

    /*
     * code=51, hex=0x33, ascii="3"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x04,  /* 000001 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=52, hex=0x34, ascii="4"
     */
    0x08,  /* 000010 */
    0x18,  /* 000110 */
    0x28,  /* 001010 */
    0x48,  /* 010010 */
    0x7C,  /* 011111 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x00,  /* 000000 */

    /*
     * code=53, hex=0x35, ascii="5"
     */
    0x7C,  /* 011111 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x04,  /* 000001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=54, hex=0x36, ascii="6"
     */
    0x18,  /* 000110 */
    0x20,  /* 001000 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=55, hex=0x37, ascii="7"
     */
    0x7C,  /* 011111 */
    0x04,  /* 000001 */
    0x08,  /* 000010 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */

    /*
     * code=56, hex=0x38, ascii="8"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=57, hex=0x39, ascii="9"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x04,  /* 000001 */
    0x08,  /* 000010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=58, hex=0x3A, ascii=":"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=59, hex=0x3B, ascii=";"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x20,  /* 001000 */

    /*
     * code=60, hex=0x3C, ascii="<"
     */
    0x08,  /* 000010 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x40,  /* 010000 */
    0x20,  /* 001000 */
    0x10,  /* 000100 */
    0x08,  /* 000010 */
    0x00,  /* 000000 */

    /*
     * code=61, hex=0x3D, ascii="="
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=62, hex=0x3E, ascii=">"
     */
    0x20,  /* 001000 */
    0x10,  /* 000100 */
    0x08,  /* 000010 */
    0x04,  /* 000001 */
    0x08,  /* 000010 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */

    /*
     * code=63, hex=0x3F, ascii="?"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x04,  /* 000001 */
    0x18,  /* 000110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=64, hex=0x40, ascii="@"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x5C,  /* 010111 */
    0x54,  /* 010101 */
    0x5C,  /* 010111 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=65, hex=0x41, ascii="A"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x7C,  /* 011111 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=66, hex=0x42, ascii="B"
     */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=67, hex=0x43, ascii="C"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=68, hex=0x44, ascii="D"
     */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=69, hex=0x45, ascii="E"
     */
    0x7C,  /* 011111 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */

    /*
     * code=70, hex=0x46, ascii="F"
     */
    0x7C,  /* 011111 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x00,  /* 000000 */

    /*
     * code=71, hex=0x47, ascii="G"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x40,  /* 010000 */
    0x5C,  /* 010111 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=72, hex=0x48, ascii="H"
     */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x7C,  /* 011111 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=73, hex=0x49, ascii="I"
     */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=74, hex=0x4A, ascii="J"
     */
    0x04,  /* 000001 */
    0x04,  /* 000001 */
    0x04,  /* 000001 */
    0x04,  /* 000001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=75, hex=0x4B, ascii="K"
     */
    0x44,  /* 010001 */
    0x48,  /* 010010 */
    0x50,  /* 010100 */
    0x60,  /* 011000 */
    0x50,  /* 010100 */
    0x48,  /* 010010 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=76, hex=0x4C, ascii="L"
     */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */

    /*
     * code=77, hex=0x4D, ascii="M"
     */
    0x44,  /* 010001 */
    0x6C,  /* 011011 */
    0x54,  /* 010101 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=78, hex=0x4E, ascii="N"
     */
    0x44,  /* 010001 */
    0x64,  /* 011001 */
    0x54,  /* 010101 */
    0x4C,  /* 010011 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=79, hex=0x4F, ascii="O"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=80, hex=0x50, ascii="P"
     */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x00,  /* 000000 */

    /*
     * code=81, hex=0x51, ascii="Q"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x54,  /* 010101 */
    0x48,  /* 010010 */
    0x34,  /* 001101 */
    0x00,  /* 000000 */

    /*
     * code=82, hex=0x52, ascii="R"
     */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x48,  /* 010010 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=83, hex=0x53, ascii="S"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=84, hex=0x54, ascii="T"
     */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=85, hex=0x55, ascii="U"
     */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=86, hex=0x56, ascii="V"
     */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x28,  /* 001010 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=87, hex=0x57, ascii="W"
     */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x54,  /* 010101 */
    0x54,  /* 010101 */
    0x54,  /* 010101 */
    0x54,  /* 010101 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=88, hex=0x58, ascii="X"
     */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x28,  /* 001010 */
    0x10,  /* 000100 */
    0x28,  /* 001010 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=89, hex=0x59, ascii="Y"
     */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x28,  /* 001010 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=90, hex=0x5A, ascii="Z"
     */
    0x78,  /* 011110 */
    0x08,  /* 000010 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=91, hex=0x5B, ascii="["
     */
    0x38,  /* 001110 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=92, hex=0x5C, ascii="\"
     */
    0x00,  /* 000000 */
    0x40,  /* 010000 */
    0x20,  /* 001000 */
    0x10,  /* 000100 */
    0x08,  /* 000010 */
    0x04,  /* 000001 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=93, hex=0x5D, ascii="]"
     */
    0x38,  /* 001110 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=94, hex=0x5E, ascii="^"
     */
    0x10,  /* 000100 */
    0x28,  /* 001010 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=95, hex=0x5F, ascii="_"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */

    /*
     * code=96, hex=0x60, ascii="`"
     */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=97, hex=0x61, ascii="a"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=98, hex=0x62, ascii="b"
     */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=99, hex=0x63, ascii="c"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x40,  /* 010000 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=100, hex=0x64, ascii="d"
     */
    0x04,  /* 000001 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=101, hex=0x65, ascii="e"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=102, hex=0x66, ascii="f"
     */
    0x18,  /* 000110 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x78,  /* 011110 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */

    /*
     * code=103, hex=0x67, ascii="g"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x04,  /* 000001 */
    0x38,  /* 001110 */

    /*
     * code=104, hex=0x68, ascii="h"
     */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x70,  /* 011100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */

    /*
     * code=105, hex=0x69, ascii="i"
     */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x00,  /* 000000 */

    /*
     * code=106, hex=0x6A, ascii="j"
     */
    0x08,  /* 000010 */
    0x00,  /* 000000 */
    0x18,  /* 000110 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */

    /*
     * code=107, hex=0x6B, ascii="k"
     */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x48,  /* 010010 */
    0x50,  /* 010100 */
    0x60,  /* 011000 */
    0x50,  /* 010100 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */

    /*
     * code=108, hex=0x6C, ascii="l"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x00,  /* 000000 */

    /*
     * code=109, hex=0x6D, ascii="m"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x68,  /* 011010 */
    0x54,  /* 010101 */
    0x54,  /* 010101 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=110, hex=0x6E, ascii="n"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x70,  /* 011100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */

    /*
     * code=111, hex=0x6F, ascii="o"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=112, hex=0x70, ascii="p"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */

    /*
     * code=113, hex=0x71, ascii="q"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x04,  /* 000001 */

    /*
     * code=114, hex=0x72, ascii="r"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x58,  /* 010110 */
    0x24,  /* 001001 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x70,  /* 011100 */
    0x00,  /* 000000 */

    /*
     * code=115, hex=0x73, ascii="s"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=116, hex=0x74, ascii="t"
     */
    0x00,  /* 000000 */
    0x20,  /* 001000 */
    0x78,  /* 011110 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x28,  /* 001010 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=117, hex=0x75, ascii="u"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x58,  /* 010110 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=118, hex=0x76, ascii="v"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x28,  /* 001010 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=119, hex=0x77, ascii="w"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x54,  /* 010101 */
    0x7C,  /* 011111 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=120, hex=0x78, ascii="x"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */

    /*
     * code=121, hex=0x79, ascii="y"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x60,  /* 011000 */

    /*
     * code=122, hex=0x7A, ascii="z"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x08,  /* 000010 */
    0x30,  /* 001100 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=123, hex=0x7B, ascii="{"
     */
    0x18,  /* 000110 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x60,  /* 011000 */
    0x20,  /* 001000 */
    0x20,  /* 001000 */
    0x18,  /* 000110 */
    0x00,  /* 000000 */

    /*
     * code=124, hex=0x7C, ascii="|"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=125, hex=0x7D, ascii="}"
     */
    0x30,  /* 001100 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x0C,  /* 000011 */
    0x08,  /* 000010 */
    0x08,  /* 000010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=126, hex=0x7E, ascii="~"
     */
    0x28,  /* 001010 */
    0x50,  /* 010100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=127, hex=0x7F, ascii="^?"
     */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x6C,  /* 011011 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=128, hex=0x80, ascii="!^@"
     */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x30,  /* 001100 */

    /*
     * code=129, hex=0x81, ascii="!^A"
     */
    0x48,  /* 010010 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x58,  /* 010110 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=130, hex=0x82, ascii="!^B"
     */
    0x0C,  /* 000011 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=131, hex=0x83, ascii="!^C"
     */
    0x38,  /* 001110 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=132, hex=0x84, ascii="!^D"
     */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=133, hex=0x85, ascii="!^E"
     */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=134, hex=0x86, ascii="!^F"
     */
    0x38,  /* 001110 */
    0x28,  /* 001010 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=135, hex=0x87, ascii="!^G"
     */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x40,  /* 010000 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x30,  /* 001100 */

    /*
     * code=136, hex=0x88, ascii="!^H"
     */
    0x38,  /* 001110 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=137, hex=0x89, ascii="!^I"
     */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=138, hex=0x8A, ascii="!^J"
     */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=139, hex=0x8B, ascii="!^K"
     */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x00,  /* 000000 */

    /*
     * code=140, hex=0x8C, ascii="!^L"
     */
    0x10,  /* 000100 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x00,  /* 000000 */

    /*
     * code=141, hex=0x8D, ascii="!^M"
     */
    0x20,  /* 001000 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x00,  /* 000000 */

    /*
     * code=142, hex=0x8E, ascii="!^N"
     */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x28,  /* 001010 */
    0x44,  /* 010001 */
    0x7C,  /* 011111 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=143, hex=0x8F, ascii="!^O"
     */
    0x38,  /* 001110 */
    0x28,  /* 001010 */
    0x38,  /* 001110 */
    0x6C,  /* 011011 */
    0x44,  /* 010001 */
    0x7C,  /* 011111 */
    0x44,  /* 010001 */
    0x00,  /* 000000 */

    /*
     * code=144, hex=0x90, ascii="!^P"
     */
    0x0C,  /* 000011 */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */

    /*
     * code=145, hex=0x91, ascii="!^Q"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x14,  /* 000101 */
    0x7C,  /* 011111 */
    0x50,  /* 010100 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=146, hex=0x92, ascii="!^R"
     */
    0x3C,  /* 001111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x7C,  /* 011111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x5C,  /* 010111 */
    0x00,  /* 000000 */

    /*
     * code=147, hex=0x93, ascii="!^S"
     */
    0x38,  /* 001110 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=148, hex=0x94, ascii="!^T"
     */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=149, hex=0x95, ascii="!^U"
     */
    0x60,  /* 011000 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=150, hex=0x96, ascii="!^V"
     */
    0x38,  /* 001110 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x58,  /* 010110 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=151, hex=0x97, ascii="!^W"
     */
    0x60,  /* 011000 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x58,  /* 010110 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=152, hex=0x98, ascii="!^X"
     */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x60,  /* 011000 */

    /*
     * code=153, hex=0x99, ascii="!^Y"
     */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=154, hex=0x9A, ascii="!^Z"
     */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=155, hex=0x9B, ascii="!^["
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=156, hex=0x9C, ascii="!^\"
     */
    0x18,  /* 000110 */
    0x24,  /* 001001 */
    0x20,  /* 001000 */
    0x78,  /* 011110 */
    0x20,  /* 001000 */
    0x24,  /* 001001 */
    0x5C,  /* 010111 */
    0x00,  /* 000000 */

    /*
     * code=157, hex=0x9D, ascii="!^]"
     */
    0x44,  /* 010001 */
    0x28,  /* 001010 */
    0x10,  /* 000100 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x7C,  /* 011111 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=158, hex=0x9E, ascii="!^^"
     */
    0x60,  /* 011000 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x68,  /* 011010 */
    0x5C,  /* 010111 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */

    /*
     * code=159, hex=0x9F, ascii="!^_"
     */
    0x08,  /* 000010 */
    0x14,  /* 000101 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x50,  /* 010100 */
    0x20,  /* 001000 */

    /*
     * code=160, hex=0xA0, ascii="! "
     */
    0x18,  /* 000110 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=161, hex=0xA1, ascii="!!"
     */
    0x18,  /* 000110 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x18,  /* 000110 */
    0x00,  /* 000000 */

    /*
     * code=162, hex=0xA2, ascii="!""
     */
    0x18,  /* 000110 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=163, hex=0xA3, ascii="!#"
     */
    0x18,  /* 000110 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x58,  /* 010110 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=164, hex=0xA4, ascii="!$"
     */
    0x28,  /* 001010 */
    0x50,  /* 010100 */
    0x00,  /* 000000 */
    0x70,  /* 011100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */

    /*
     * code=165, hex=0xA5, ascii="!%"
     */
    0x28,  /* 001010 */
    0x50,  /* 010100 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x68,  /* 011010 */
    0x58,  /* 010110 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */

    /*
     * code=166, hex=0xA6, ascii="!&"
     */
    0x38,  /* 001110 */
    0x04,  /* 000001 */
    0x3C,  /* 001111 */
    0x44,  /* 010001 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */
    0x3C,  /* 001111 */
    0x00,  /* 000000 */

    /*
     * code=167, hex=0xA7, ascii="!'"
     */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=168, hex=0xA8, ascii="!("
     */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x30,  /* 001100 */
    0x40,  /* 010000 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=169, hex=0xA9, ascii="!)"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=170, hex=0xAA, ascii="!*"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x04,  /* 000001 */
    0x04,  /* 000001 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=171, hex=0xAB, ascii="!+"
     */
    0x40,  /* 010000 */
    0x48,  /* 010010 */
    0x50,  /* 010100 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x08,  /* 000010 */
    0x1C,  /* 000111 */
    0x00,  /* 000000 */

    /*
     * code=172, hex=0xAC, ascii="!,"
     */
    0x40,  /* 010000 */
    0x48,  /* 010010 */
    0x50,  /* 010100 */
    0x2C,  /* 001011 */
    0x54,  /* 010101 */
    0x1C,  /* 000111 */
    0x04,  /* 000001 */
    0x00,  /* 000000 */

    /*
     * code=173, hex=0xAD, ascii="!-"
     */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=174, hex=0xAE, ascii="!."
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x24,  /* 001001 */
    0x48,  /* 010010 */
    0x24,  /* 001001 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=175, hex=0xAF, ascii="!/"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x24,  /* 001001 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=176, hex=0xB0, ascii="!0"
     */
    0x54,  /* 010101 */
    0x00,  /* 000000 */
    0xA8,  /* 101010 */
    0x00,  /* 000000 */
    0x54,  /* 010101 */
    0x00,  /* 000000 */
    0xA8,  /* 101010 */
    0x00,  /* 000000 */

    /*
     * code=177, hex=0xB1, ascii="!1"
     */
    0x54,  /* 010101 */
    0xA8,  /* 101010 */
    0x54,  /* 010101 */
    0xA8,  /* 101010 */
    0x54,  /* 010101 */
    0xA8,  /* 101010 */
    0x54,  /* 010101 */
    0xA8,  /* 101010 */

    /*
     * code=178, hex=0xB2, ascii="!2"
     */
    0xA8,  /* 101010 */
    0xFC,  /* 111111 */
    0x54,  /* 010101 */
    0xFC,  /* 111111 */
    0xA8,  /* 101010 */
    0xFC,  /* 111111 */
    0x54,  /* 010101 */
    0xFC,  /* 111111 */

    /*
     * code=179, hex=0xB3, ascii="!3"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=180, hex=0xB4, ascii="!4"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=181, hex=0xB5, ascii="!5"
     */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=182, hex=0xB6, ascii="!6"
     */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0xD0,  /* 110100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=183, hex=0xB7, ascii="!7"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xF0,  /* 111100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=184, hex=0xB8, ascii="!8"
     */
    0x00,  /* 000000 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=185, hex=0xB9, ascii="!9"
     */
    0x50,  /* 010100 */
    0xD0,  /* 110100 */
    0x10,  /* 000100 */
    0xD0,  /* 110100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=186, hex=0xBA, ascii="!:"
     */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=187, hex=0xBB, ascii="!;"
     */
    0x00,  /* 000000 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0xD0,  /* 110100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=188, hex=0xBC, ascii="!<"
     */
    0x50,  /* 010100 */
    0xD0,  /* 110100 */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=189, hex=0xBD, ascii="!="
     */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0xF0,  /* 111100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=190, hex=0xBE, ascii="!>"
     */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=191, hex=0xBF, ascii="!?"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xF0,  /* 111100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=192, hex=0xC0, ascii="!@"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x1C,  /* 000111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=193, hex=0xC1, ascii="!A"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=194, hex=0xC2, ascii="!B"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=195, hex=0xC3, ascii="!C"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=196, hex=0xC4, ascii="!D"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=197, hex=0xC5, ascii="!E"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0xFC,  /* 111111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=198, hex=0xC6, ascii="!F"
     */
    0x10,  /* 000100 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=199, hex=0xC7, ascii="!G"
     */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x5C,  /* 010111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=200, hex=0xC8, ascii="!H"
     */
    0x50,  /* 010100 */
    0x5C,  /* 010111 */
    0x40,  /* 010000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=201, hex=0xC9, ascii="!I"
     */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x40,  /* 010000 */
    0x5C,  /* 010111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=202, hex=0xCA, ascii="!J"
     */
    0x50,  /* 010100 */
    0xDC,  /* 110111 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=203, hex=0xCB, ascii="!K"
     */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0xDC,  /* 110111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=204, hex=0xCC, ascii="!L"
     */
    0x50,  /* 010100 */
    0x5C,  /* 010111 */
    0x40,  /* 010000 */
    0x5C,  /* 010111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=205, hex=0xCD, ascii="!M"
     */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=206, hex=0xCE, ascii="!N"
     */
    0x50,  /* 010100 */
    0xDC,  /* 110111 */
    0x00,  /* 000000 */
    0xDC,  /* 110111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=207, hex=0xCF, ascii="!O"
     */
    0x10,  /* 000100 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=208, hex=0xD0, ascii="!P"
     */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=209, hex=0xD1, ascii="!Q"
     */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=210, hex=0xD2, ascii="!R"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=211, hex=0xD3, ascii="!S"
     */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=212, hex=0xD4, ascii="!T"
     */
    0x10,  /* 000100 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x1C,  /* 000111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=213, hex=0xD5, ascii="!U"
     */
    0x00,  /* 000000 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=214, hex=0xD6, ascii="!V"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=215, hex=0xD7, ascii="!W"
     */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0xDC,  /* 110111 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */

    /*
     * code=216, hex=0xD8, ascii="!X"
     */
    0x10,  /* 000100 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=217, hex=0xD9, ascii="!Y"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0xF0,  /* 111100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=218, hex=0xDA, ascii="!Z"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=219, hex=0xDB, ascii="!["
     */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */

    /*
     * code=220, hex=0xDC, ascii="!\"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */

    /*
     * code=221, hex=0xDD, ascii="!]"
     */
    0xE0,  /* 111000 */
    0xE0,  /* 111000 */
    0xE0,  /* 111000 */
    0xE0,  /* 111000 */
    0xE0,  /* 111000 */
    0xE0,  /* 111000 */
    0xE0,  /* 111000 */
    0xE0,  /* 111000 */

    /*
     * code=222, hex=0xDE, ascii="!^"
     */
    0x1C,  /* 000111 */
    0x1C,  /* 000111 */
    0x1C,  /* 000111 */
    0x1C,  /* 000111 */
    0x1C,  /* 000111 */
    0x1C,  /* 000111 */
    0x1C,  /* 000111 */
    0x1C,  /* 000111 */

    /*
     * code=223, hex=0xDF, ascii="!_"
     */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0xFC,  /* 111111 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=224, hex=0xE0, ascii="!`"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x34,  /* 001101 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x34,  /* 001101 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=225, hex=0xE1, ascii="!a"
     */
    0x00,  /* 000000 */
    0x70,  /* 011100 */
    0x48,  /* 010010 */
    0x70,  /* 011100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x70,  /* 011100 */
    0x40,  /* 010000 */

    /*
     * code=226, hex=0xE2, ascii="!b"
     */
    0x78,  /* 011110 */
    0x48,  /* 010010 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */
    0x00,  /* 000000 */

    /*
     * code=227, hex=0xE3, ascii="!c"
     */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */

    /*
     * code=228, hex=0xE4, ascii="!d"
     */
    0x78,  /* 011110 */
    0x48,  /* 010010 */
    0x20,  /* 001000 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x48,  /* 010010 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=229, hex=0xE5, ascii="!e"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x3C,  /* 001111 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=230, hex=0xE6, ascii="!f"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x70,  /* 011100 */
    0x40,  /* 010000 */
    0x40,  /* 010000 */

    /*
     * code=231, hex=0xE7, ascii="!g"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x50,  /* 010100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=232, hex=0xE8, ascii="!h"
     */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */

    /*
     * code=233, hex=0xE9, ascii="!i"
     */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x78,  /* 011110 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=234, hex=0xEA, ascii="!j"
     */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x44,  /* 010001 */
    0x44,  /* 010001 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x6C,  /* 011011 */
    0x00,  /* 000000 */

    /*
     * code=235, hex=0xEB, ascii="!k"
     */
    0x30,  /* 001100 */
    0x40,  /* 010000 */
    0x20,  /* 001000 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */

    /*
     * code=236, hex=0xEC, ascii="!l"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x54,  /* 010101 */
    0x54,  /* 010101 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=237, hex=0xED, ascii="!m"
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x54,  /* 010101 */
    0x54,  /* 010101 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */

    /*
     * code=238, hex=0xEE, ascii="!n"
     */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x40,  /* 010000 */
    0x78,  /* 011110 */
    0x40,  /* 010000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=239, hex=0xEF, ascii="!o"
     */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=240, hex=0xF0, ascii="!p"
     */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=241, hex=0xF1, ascii="!q"
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x38,  /* 001110 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x38,  /* 001110 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=242, hex=0xF2, ascii="!r"
     */
    0x40,  /* 010000 */
    0x30,  /* 001100 */
    0x08,  /* 000010 */
    0x30,  /* 001100 */
    0x40,  /* 010000 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=243, hex=0xF3, ascii="!s"
     */
    0x08,  /* 000010 */
    0x30,  /* 001100 */
    0x40,  /* 010000 */
    0x30,  /* 001100 */
    0x08,  /* 000010 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */

    /*
     * code=244, hex=0xF4, ascii="!t"
     */
    0x00,  /* 000000 */
    0x08,  /* 000010 */
    0x14,  /* 000101 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */

    /*
     * code=245, hex=0xF5, ascii="!u"
     */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x50,  /* 010100 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */

    /*
     * code=246, hex=0xF6, ascii="!v"
     */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x7C,  /* 011111 */
    0x00,  /* 000000 */
    0x10,  /* 000100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=247, hex=0xF7, ascii="!w"
     */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x50,  /* 010100 */
    0x00,  /* 000000 */
    0x28,  /* 001010 */
    0x50,  /* 010100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=248, hex=0xF8, ascii="!x"
     */
    0x30,  /* 001100 */
    0x48,  /* 010010 */
    0x48,  /* 010010 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=249, hex=0xF9, ascii="!y"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x30,  /* 001100 */
    0x30,  /* 001100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=250, hex=0xFA, ascii="!z"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=251, hex=0xFB, ascii="!{"
     */
    0x00,  /* 000000 */
    0x1C,  /* 000111 */
    0x10,  /* 000100 */
    0x10,  /* 000100 */
    0x50,  /* 010100 */
    0x50,  /* 010100 */
    0x20,  /* 001000 */
    0x00,  /* 000000 */

    /*
     * code=252, hex=0xFC, ascii="!|"
     */
    0x50,  /* 010100 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x28,  /* 001010 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=253, hex=0xFD, ascii="!}"
     */
    0x60,  /* 011000 */
    0x10,  /* 000100 */
    0x20,  /* 001000 */
    0x70,  /* 011100 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=254, hex=0xFE, ascii="!~"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x78,  /* 011110 */
    0x78,  /* 011110 */
    0x78,  /* 011110 */
    0x78,  /* 011110 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */

    /*
     * code=255, hex=0xFF, ascii="!^ź"
     */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00,  /* 000000 */
    0x00   /* 000000 */
};

static const unsigned char console_font_5x8[] PROGMEM = {

    /*
     * code=0, hex=0x00, ascii="^@"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=1, hex=0x01, ascii="^A"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0xA8,  /* 10101 */
    0xF8,  /* 11111 */
    0xD8,  /* 11011 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=2, hex=0x02, ascii="^B"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0xA8,  /* 10101 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=3, hex=0x03, ascii="^C"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=4, hex=0x04, ascii="^D"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0xF8,  /* 11111 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=5, hex=0x05, ascii="^E"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0xA8,  /* 10101 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=6, hex=0x06, ascii="^F"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0xF8,  /* 11111 */
    0xA8,  /* 10101 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=7, hex=0x07, ascii="^G"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=8, hex=0x08, ascii="^H"
     */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xD8,  /* 11011 */
    0x88,  /* 10001 */
    0xD8,  /* 11011 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */

    /*
     * code=9, hex=0x09, ascii="^I"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=10, hex=0x0A, ascii="^J"
     */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xD8,  /* 11011 */
    0x88,  /* 10001 */
    0xD8,  /* 11011 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */

    /*
     * code=11, hex=0x0B, ascii="^K"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x38,  /* 00111 */
    0x18,  /* 00011 */
    0x68,  /* 01101 */
    0xA0,  /* 10100 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=12, hex=0x0C, ascii="^L"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=13, hex=0x0D, ascii="^M"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x40,  /* 01000 */
    0xC0,  /* 11000 */
    0x80,  /* 10000 */
    0x00,  /* 00000 */

    /*
     * code=14, hex=0x0E, ascii="^N"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x38,  /* 00111 */
    0x48,  /* 01001 */
    0x58,  /* 01011 */
    0xD0,  /* 11010 */
    0x80,  /* 10000 */
    0x00,  /* 00000 */

    /*
     * code=15, hex=0x0F, ascii="^O"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=16, hex=0x10, ascii="^P"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x60,  /* 01100 */
    0x70,  /* 01110 */
    0x60,  /* 01100 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=17, hex=0x11, ascii="^Q"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x30,  /* 00110 */
    0x70,  /* 01110 */
    0x30,  /* 00110 */
    0x10,  /* 00010 */
    0x00,  /* 00000 */

    /*
     * code=18, hex=0x12, ascii="^R"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=19, hex=0x13, ascii="^S"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=20, hex=0x14, ascii="^T"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x78,  /* 01111 */
    0xD0,  /* 11010 */
    0xD0,  /* 11010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=21, hex=0x15, ascii="^U"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x18,  /* 00011 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x48,  /* 01001 */
    0x30,  /* 00110 */
    0xC0,  /* 11000 */

    /*
     * code=22, hex=0x16, ascii="^V"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */

    /*
     * code=23, hex=0x17, ascii="^W"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */

    /*
     * code=24, hex=0x18, ascii="^X"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=25, hex=0x19, ascii="^Y"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=26, hex=0x1A, ascii="^Z"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0xF8,  /* 11111 */
    0x10,  /* 00010 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=27, hex=0x1B, ascii="^["
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0xF8,  /* 11111 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=28, hex=0x1C, ascii="^\"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x80,  /* 10000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=29, hex=0x1D, ascii="^]"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0xF8,  /* 11111 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=30, hex=0x1E, ascii="^^"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */

    /*
     * code=31, hex=0x1F, ascii="^_"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=32, hex=0x20, ascii=" "
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=33, hex=0x21, ascii="!"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=34, hex=0x22, ascii="""
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=35, hex=0x23, ascii="#"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0xF8,  /* 11111 */
    0x50,  /* 01010 */
    0xF8,  /* 11111 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=36, hex=0x24, ascii="$"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x30,  /* 00110 */
    0x40,  /* 01000 */
    0x30,  /* 00110 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */

    /*
     * code=37, hex=0x25, ascii="%"
     */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0xA8,  /* 10101 */
    0x50,  /* 01010 */
    0x30,  /* 00110 */
    0x68,  /* 01101 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=38, hex=0x26, ascii="&"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x30,  /* 00110 */
    0x40,  /* 01000 */
    0x68,  /* 01101 */
    0x90,  /* 10010 */
    0x68,  /* 01101 */
    0x00,  /* 00000 */

    /*
     * code=39, hex=0x27, ascii="'"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=40, hex=0x28, ascii="("
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=41, hex=0x29, ascii=")"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=42, hex=0x2A, ascii="*"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=43, hex=0x2B, ascii="+"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=44, hex=0x2C, ascii=","
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */

    /*
     * code=45, hex=0x2D, ascii="-"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=46, hex=0x2E, ascii="."
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=47, hex=0x2F, ascii="/"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=48, hex=0x30, ascii="0"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=49, hex=0x31, ascii="1"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=50, hex=0x32, ascii="2"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=51, hex=0x33, ascii="3"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x10,  /* 00010 */
    0x60,  /* 01100 */
    0x10,  /* 00010 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */

    /*
     * code=52, hex=0x34, ascii="4"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x30,  /* 00110 */
    0x50,  /* 01010 */
    0xF0,  /* 11110 */
    0x10,  /* 00010 */
    0x00,  /* 00000 */

    /*
     * code=53, hex=0x35, ascii="5"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0xE0,  /* 11100 */
    0x10,  /* 00010 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */

    /*
     * code=54, hex=0x36, ascii="6"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x80,  /* 10000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=55, hex=0x37, ascii="7"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=56, hex=0x38, ascii="8"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=57, hex=0x39, ascii="9"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x10,  /* 00010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=58, hex=0x3A, ascii=":"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=59, hex=0x3B, ascii=";"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */

    /*
     * code=60, hex=0x3C, ascii="<"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x10,  /* 00010 */
    0x00,  /* 00000 */

    /*
     * code=61, hex=0x3D, ascii="="
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=62, hex=0x3E, ascii=">"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=63, hex=0x3F, ascii="?"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x10,  /* 00010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=64, hex=0x40, ascii="@"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x88,  /* 10001 */
    0xB0,  /* 10110 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=65, hex=0x41, ascii="A"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=66, hex=0x42, ascii="B"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */

    /*
     * code=67, hex=0x43, ascii="C"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=68, hex=0x44, ascii="D"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */

    /*
     * code=69, hex=0x45, ascii="E"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0xE0,  /* 11100 */
    0x80,  /* 10000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=70, hex=0x46, ascii="F"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0xE0,  /* 11100 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x00,  /* 00000 */

    /*
     * code=71, hex=0x47, ascii="G"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x80,  /* 10000 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=72, hex=0x48, ascii="H"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=73, hex=0x49, ascii="I"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=74, hex=0x4A, ascii="J"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x10,  /* 00010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=75, hex=0x4B, ascii="K"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0xA0,  /* 10100 */
    0xC0,  /* 11000 */
    0xA0,  /* 10100 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=76, hex=0x4C, ascii="L"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=77, hex=0x4D, ascii="M"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=78, hex=0x4E, ascii="N"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0xD0,  /* 11010 */
    0xB0,  /* 10110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=79, hex=0x4F, ascii="O"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=80, hex=0x50, ascii="P"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x00,  /* 00000 */

    /*
     * code=81, hex=0x51, ascii="Q"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x10,  /* 00010 */

    /*
     * code=82, hex=0x52, ascii="R"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=83, hex=0x53, ascii="S"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x80,  /* 10000 */
    0x60,  /* 01100 */
    0x10,  /* 00010 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */

    /*
     * code=84, hex=0x54, ascii="T"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=85, hex=0x55, ascii="U"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=86, hex=0x56, ascii="V"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=87, hex=0x57, ascii="W"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x88,  /* 10001 */
    0xA8,  /* 10101 */
    0xA8,  /* 10101 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=88, hex=0x58, ascii="X"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x50,  /* 01010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=89, hex=0x59, ascii="Y"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=90, hex=0x5A, ascii="Z"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x80,  /* 10000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=91, hex=0x5B, ascii="["
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=92, hex=0x5C, ascii="\"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x10,  /* 00010 */
    0x00,  /* 00000 */

    /*
     * code=93, hex=0x5D, ascii="]"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=94, hex=0x5E, ascii="^"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=95, hex=0x5F, ascii="_"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */

    /*
     * code=96, hex=0x60, ascii="`"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=97, hex=0x61, ascii="a"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x10,  /* 00010 */
    0x70,  /* 01110 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=98, hex=0x62, ascii="b"
     */
    0x00,  /* 00000 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */

    /*
     * code=99, hex=0x63, ascii="c"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x30,  /* 00110 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x30,  /* 00110 */
    0x00,  /* 00000 */

    /*
     * code=100, hex=0x64, ascii="d"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x70,  /* 01110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=101, hex=0x65, ascii="e"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=102, hex=0x66, ascii="f"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x30,  /* 00110 */
    0x40,  /* 01000 */
    0xE0,  /* 11100 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=103, hex=0x67, ascii="g"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x10,  /* 00010 */
    0x60,  /* 01100 */

    /*
     * code=104, hex=0x68, ascii="h"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x80,  /* 10000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=105, hex=0x69, ascii="i"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=106, hex=0x6A, ascii="j"
     */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x10,  /* 00010 */
    0x10,  /* 00010 */
    0x10,  /* 00010 */
    0x60,  /* 01100 */

    /*
     * code=107, hex=0x6B, ascii="k"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x80,  /* 10000 */
    0xA0,  /* 10100 */
    0xC0,  /* 11000 */
    0xA0,  /* 10100 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=108, hex=0x6C, ascii="l"
     */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=109, hex=0x6D, ascii="m"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=110, hex=0x6E, ascii="n"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=111, hex=0x6F, ascii="o"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=112, hex=0x70, ascii="p"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x80,  /* 10000 */

    /*
     * code=113, hex=0x71, ascii="q"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x10,  /* 00010 */

    /*
     * code=114, hex=0x72, ascii="r"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x60,  /* 01100 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=115, hex=0x73, ascii="s"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0xC0,  /* 11000 */
    0x30,  /* 00110 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */

    /*
     * code=116, hex=0x74, ascii="t"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0xF0,  /* 11110 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x30,  /* 00110 */
    0x00,  /* 00000 */

    /*
     * code=117, hex=0x75, ascii="u"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=118, hex=0x76, ascii="v"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=119, hex=0x77, ascii="w"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=120, hex=0x78, ascii="x"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=121, hex=0x79, ascii="y"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x10,  /* 00010 */
    0x60,  /* 01100 */

    /*
     * code=122, hex=0x7A, ascii="z"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=123, hex=0x7B, ascii="{"
     */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x10,  /* 00010 */
    0x00,  /* 00000 */

    /*
     * code=124, hex=0x7C, ascii="|"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=125, hex=0x7D, ascii="}"
     */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x10,  /* 00010 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=126, hex=0x7E, ascii="~"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0xA0,  /* 10100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=127, hex=0x7F, ascii="^?"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x88,  /* 10001 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */

    /*
     * code=128, hex=0x80, ascii="!^@"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */

    /*
     * code=129, hex=0x81, ascii="!^A"
     */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=130, hex=0x82, ascii="!^B"
     */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=131, hex=0x83, ascii="!^C"
     */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0xC0,  /* 11000 */
    0x20,  /* 00100 */
    0xA0,  /* 10100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=132, hex=0x84, ascii="!^D"
     */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0xC0,  /* 11000 */
    0x20,  /* 00100 */
    0x60,  /* 01100 */
    0xB0,  /* 10110 */
    0x00,  /* 00000 */

    /*
     * code=133, hex=0x85, ascii="!^E"
     */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0xC0,  /* 11000 */
    0x20,  /* 00100 */
    0x60,  /* 01100 */
    0xB0,  /* 10110 */
    0x00,  /* 00000 */

    /*
     * code=134, hex=0x86, ascii="!^F"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0xC0,  /* 11000 */
    0x20,  /* 00100 */
    0x60,  /* 01100 */
    0xB0,  /* 10110 */
    0x00,  /* 00000 */

    /*
     * code=135, hex=0x87, ascii="!^G"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x30,  /* 00110 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x30,  /* 00110 */
    0x20,  /* 00100 */

    /*
     * code=136, hex=0x88, ascii="!^H"
     */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=137, hex=0x89, ascii="!^I"
     */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=138, hex=0x8A, ascii="!^J"
     */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=139, hex=0x8B, ascii="!^K"
     */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=140, hex=0x8C, ascii="!^L"
     */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=141, hex=0x8D, ascii="!^M"
     */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=142, hex=0x8E, ascii="!^N"
     */
    0xA0,  /* 10100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=143, hex=0x8F, ascii="!^O"
     */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=144, hex=0x90, ascii="!^P"
     */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0xF0,  /* 11110 */
    0x80,  /* 10000 */
    0xE0,  /* 11100 */
    0x80,  /* 10000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=145, hex=0x91, ascii="!^Q"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xD8,  /* 11011 */
    0x78,  /* 01111 */
    0xE0,  /* 11100 */
    0xB8,  /* 10111 */
    0x00,  /* 00000 */

    /*
     * code=146, hex=0x92, ascii="!^R"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0xA0,  /* 10100 */
    0xF0,  /* 11110 */
    0xA0,  /* 10100 */
    0xB0,  /* 10110 */
    0x00,  /* 00000 */

    /*
     * code=147, hex=0x93, ascii="!^S"
     */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=148, hex=0x94, ascii="!^T"
     */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=149, hex=0x95, ascii="!^U"
     */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=150, hex=0x96, ascii="!^V"
     */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=151, hex=0x97, ascii="!^W"
     */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=152, hex=0x98, ascii="!^X"
     */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x10,  /* 00010 */
    0x60,  /* 01100 */

    /*
     * code=153, hex=0x99, ascii="!^Y"
     */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=154, hex=0x9A, ascii="!^Z"
     */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=155, hex=0x9B, ascii="!^["
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x80,  /* 10000 */
    0x80,  /* 10000 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */

    /*
     * code=156, hex=0x9C, ascii="!^\"
     */
    0x00,  /* 00000 */
    0x30,  /* 00110 */
    0x50,  /* 01010 */
    0x40,  /* 01000 */
    0xE0,  /* 11100 */
    0x40,  /* 01000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=157, hex=0x9D, ascii="!^]"
     */
    0x00,  /* 00000 */
    0xD8,  /* 11011 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=158, hex=0x9E, ascii="!^^"
     */
    0x00,  /* 00000 */
    0xC0,  /* 11000 */
    0xA0,  /* 10100 */
    0xB0,  /* 10110 */
    0xF8,  /* 11111 */
    0x90,  /* 10010 */
    0x88,  /* 10001 */
    0x00,  /* 00000 */

    /*
     * code=159, hex=0x9F, ascii="!^_"
     */
    0x00,  /* 00000 */
    0x30,  /* 00110 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0xF0,  /* 11110 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x80,  /* 10000 */

    /*
     * code=160, hex=0xA0, ascii="! "
     */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */
    0xC0,  /* 11000 */
    0x20,  /* 00100 */
    0x60,  /* 01100 */
    0xB0,  /* 10110 */
    0x00,  /* 00000 */

    /*
     * code=161, hex=0xA1, ascii="!!"
     */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=162, hex=0xA2, ascii="!""
     */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=163, hex=0xA3, ascii="!#"
     */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=164, hex=0xA4, ascii="!$"
     */
    0x50,  /* 01010 */
    0xA0,  /* 10100 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=165, hex=0xA5, ascii="!%"
     */
    0x50,  /* 01010 */
    0xA0,  /* 10100 */
    0x90,  /* 10010 */
    0xD0,  /* 11010 */
    0xD0,  /* 11010 */
    0xB0,  /* 10110 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=166, hex=0xA6, ascii="!&"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x30,  /* 00110 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=167, hex=0xA7, ascii="!'"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=168, hex=0xA8, ascii="!("
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=169, hex=0xA9, ascii="!)"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x80,  /* 10000 */
    0x00,  /* 00000 */

    /*
     * code=170, hex=0xAA, ascii="!*"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x08,  /* 00001 */
    0x00,  /* 00000 */

    /*
     * code=171, hex=0xAB, ascii="!+"
     */
    0x00,  /* 00000 */
    0x80,  /* 10000 */
    0x90,  /* 10010 */
    0xA0,  /* 10100 */
    0x58,  /* 01011 */
    0x88,  /* 10001 */
    0x38,  /* 00111 */
    0x00,  /* 00000 */

    /*
     * code=172, hex=0xAC, ascii="!,"
     */
    0x00,  /* 00000 */
    0x88,  /* 10001 */
    0x90,  /* 10010 */
    0xA0,  /* 10100 */
    0x48,  /* 01001 */
    0x98,  /* 10011 */
    0x38,  /* 00111 */
    0x08,  /* 00001 */

    /*
     * code=173, hex=0xAD, ascii="!-"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=174, hex=0xAE, ascii="!."
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0xA0,  /* 10100 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=175, hex=0xAF, ascii="!/"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xA0,  /* 10100 */
    0x50,  /* 01010 */
    0xA0,  /* 10100 */
    0x00,  /* 00000 */

    /*
     * code=176, hex=0xB0, ascii="!0"
     */
    0xA8,  /* 10101 */
    0x50,  /* 01010 */
    0xA8,  /* 10101 */
    0x50,  /* 01010 */
    0xA8,  /* 10101 */
    0x50,  /* 01010 */
    0xA8,  /* 10101 */
    0x50,  /* 01010 */

    /*
     * code=177, hex=0xB1, ascii="!1"
     */
    0xE8,  /* 11101 */
    0x50,  /* 01010 */
    0xB8,  /* 10111 */
    0x50,  /* 01010 */
    0xE8,  /* 11101 */
    0x50,  /* 01010 */
    0xB8,  /* 10111 */
    0x50,  /* 01010 */

    /*
     * code=178, hex=0xB2, ascii="!2"
     */
    0xD8,  /* 11011 */
    0x70,  /* 01110 */
    0xD8,  /* 11011 */
    0x70,  /* 01110 */
    0xD8,  /* 11011 */
    0x70,  /* 01110 */
    0xD8,  /* 11011 */
    0x70,  /* 01110 */

    /*
     * code=179, hex=0xB3, ascii="!3"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=180, hex=0xB4, ascii="!4"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xE0,  /* 11100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=181, hex=0xB5, ascii="!5"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xE0,  /* 11100 */
    0x20,  /* 00100 */
    0xE0,  /* 11100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=182, hex=0xB6, ascii="!6"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xD0,  /* 11010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=183, hex=0xB7, ascii="!7"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=184, hex=0xB8, ascii="!8"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x20,  /* 00100 */
    0xE0,  /* 11100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=185, hex=0xB9, ascii="!9"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xD0,  /* 11010 */
    0x10,  /* 00010 */
    0xD0,  /* 11010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=186, hex=0xBA, ascii="!:"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=187, hex=0xBB, ascii="!;"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x10,  /* 00010 */
    0xD0,  /* 11010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=188, hex=0xBC, ascii="!<"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xD0,  /* 11010 */
    0x10,  /* 00010 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=189, hex=0xBD, ascii="!="
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=190, hex=0xBE, ascii="!>"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xE0,  /* 11100 */
    0x20,  /* 00100 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=191, hex=0xBF, ascii="!?"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xE0,  /* 11100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=192, hex=0xC0, ascii="!@"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x38,  /* 00111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=193, hex=0xC1, ascii="!A"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=194, hex=0xC2, ascii="!B"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=195, hex=0xC3, ascii="!C"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x38,  /* 00111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=196, hex=0xC4, ascii="!D"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=197, hex=0xC5, ascii="!E"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=198, hex=0xC6, ascii="!F"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x38,  /* 00111 */
    0x20,  /* 00100 */
    0x38,  /* 00111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=199, hex=0xC7, ascii="!G"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x58,  /* 01011 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=200, hex=0xC8, ascii="!H"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x58,  /* 01011 */
    0x40,  /* 01000 */
    0x78,  /* 01111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=201, hex=0xC9, ascii="!I"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x78,  /* 01111 */
    0x40,  /* 01000 */
    0x58,  /* 01011 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=202, hex=0xCA, ascii="!J"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xD8,  /* 11011 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=203, hex=0xCB, ascii="!K"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0xD8,  /* 11011 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=204, hex=0xCC, ascii="!L"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x58,  /* 01011 */
    0x40,  /* 01000 */
    0x58,  /* 01011 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=205, hex=0xCD, ascii="!M"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=206, hex=0xCE, ascii="!N"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xD8,  /* 11011 */
    0x00,  /* 00000 */
    0xD8,  /* 11011 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=207, hex=0xCF, ascii="!O"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=208, hex=0xD0, ascii="!P"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=209, hex=0xD1, ascii="!Q"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=210, hex=0xD2, ascii="!R"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=211, hex=0xD3, ascii="!S"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x78,  /* 01111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=212, hex=0xD4, ascii="!T"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x38,  /* 00111 */
    0x20,  /* 00100 */
    0x38,  /* 00111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=213, hex=0xD5, ascii="!U"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x38,  /* 00111 */
    0x20,  /* 00100 */
    0x38,  /* 00111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=214, hex=0xD6, ascii="!V"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x78,  /* 01111 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=215, hex=0xD7, ascii="!W"
     */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0xF8,  /* 11111 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */

    /*
     * code=216, hex=0xD8, ascii="!X"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=217, hex=0xD9, ascii="!Y"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xE0,  /* 11100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=218, hex=0xDA, ascii="!Z"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x38,  /* 00111 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=219, hex=0xDB, ascii="!["
     */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */

    /*
     * code=220, hex=0xDC, ascii="!\"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */

    /*
     * code=221, hex=0xDD, ascii="!]"
     */
    0xE0,  /* 11100 */
    0xE0,  /* 11100 */
    0xE0,  /* 11100 */
    0xE0,  /* 11100 */
    0xE0,  /* 11100 */
    0xE0,  /* 11100 */
    0xE0,  /* 11100 */
    0xE0,  /* 11100 */

    /*
     * code=222, hex=0xDE, ascii="!^"
     */
    0x18,  /* 00011 */
    0x18,  /* 00011 */
    0x18,  /* 00011 */
    0x18,  /* 00011 */
    0x18,  /* 00011 */
    0x18,  /* 00011 */
    0x18,  /* 00011 */
    0x18,  /* 00011 */

    /*
     * code=223, hex=0xDF, ascii="!_"
     */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=224, hex=0xE0, ascii="!`"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x68,  /* 01101 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x68,  /* 01101 */
    0x00,  /* 00000 */

    /*
     * code=225, hex=0xE1, ascii="!a"
     */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0xF0,  /* 11110 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0xE0,  /* 11100 */
    0x80,  /* 10000 */

    /*
     * code=226, hex=0xE2, ascii="!b"
     */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=227, hex=0xE3, ascii="!c"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */

    /*
     * code=228, hex=0xE4, ascii="!d"
     */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x48,  /* 01001 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x88,  /* 10001 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */

    /*
     * code=229, hex=0xE5, ascii="!e"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x78,  /* 01111 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=230, hex=0xE6, ascii="!f"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0xE8,  /* 11101 */
    0x80,  /* 10000 */

    /*
     * code=231, hex=0xE7, ascii="!g"
     */
    0x00,  /* 00000 */
    0x98,  /* 10011 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */

    /*
     * code=232, hex=0xE8, ascii="!h"
     */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x88,  /* 10001 */
    0x70,  /* 01110 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=233, hex=0xE9, ascii="!i"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x88,  /* 10001 */
    0xF8,  /* 11111 */
    0x88,  /* 10001 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=234, hex=0xEA, ascii="!j"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x88,  /* 10001 */
    0x88,  /* 10001 */
    0x50,  /* 01010 */
    0xD8,  /* 11011 */
    0x00,  /* 00000 */

    /*
     * code=235, hex=0xEB, ascii="!k"
     */
    0x60,  /* 01100 */
    0x80,  /* 10000 */
    0x40,  /* 01000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=236, hex=0xEC, ascii="!l"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0xA8,  /* 10101 */
    0xA8,  /* 10101 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=237, hex=0xED, ascii="!m"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x08,  /* 00001 */
    0x70,  /* 01110 */
    0xA8,  /* 10101 */
    0x48,  /* 01001 */
    0xB0,  /* 10110 */
    0x00,  /* 00000 */

    /*
     * code=238, hex=0xEE, ascii="!n"
     */
    0x00,  /* 00000 */
    0x30,  /* 00110 */
    0x40,  /* 01000 */
    0x70,  /* 01110 */
    0x40,  /* 01000 */
    0x40,  /* 01000 */
    0x30,  /* 00110 */
    0x00,  /* 00000 */

    /*
     * code=239, hex=0xEF, ascii="!o"
     */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x90,  /* 10010 */
    0x00,  /* 00000 */

    /*
     * code=240, hex=0xF0, ascii="!p"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=241, hex=0xF1, ascii="!q"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0xF8,  /* 11111 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0xF8,  /* 11111 */
    0x00,  /* 00000 */

    /*
     * code=242, hex=0xF2, ascii="!r"
     */
    0x00,  /* 00000 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */

    /*
     * code=243, hex=0xF3, ascii="!s"
     */
    0x00,  /* 00000 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x40,  /* 01000 */
    0x20,  /* 00100 */
    0x10,  /* 00010 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */

    /*
     * code=244, hex=0xF4, ascii="!t"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x18,  /* 00011 */
    0x28,  /* 00101 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */

    /*
     * code=245, hex=0xF5, ascii="!u"
     */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0x20,  /* 00100 */
    0xA0,  /* 10100 */
    0xC0,  /* 11000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=246, hex=0xF6, ascii="!v"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */
    0xF0,  /* 11110 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */

    /*
     * code=247, hex=0xF7, ascii="!w"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0xA0,  /* 10100 */
    0x00,  /* 00000 */
    0x50,  /* 01010 */
    0xA0,  /* 10100 */
    0x00,  /* 00000 */

    /*
     * code=248, hex=0xF8, ascii="!x"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x50,  /* 01010 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=249, hex=0xF9, ascii="!y"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x60,  /* 01100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=250, hex=0xFA, ascii="!z"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x20,  /* 00100 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=251, hex=0xFB, ascii="!{"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x18,  /* 00011 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0xA0,  /* 10100 */
    0x40,  /* 01000 */
    0x00,  /* 00000 */

    /*
     * code=252, hex=0xFC, ascii="!|"
     */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x50,  /* 01010 */
    0x50,  /* 01010 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=253, hex=0xFD, ascii="!}"
     */
    0x00,  /* 00000 */
    0x60,  /* 01100 */
    0x10,  /* 00010 */
    0x20,  /* 00100 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=254, hex=0xFE, ascii="!~"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x70,  /* 01110 */
    0x70,  /* 01110 */
    0x70,  /* 01110 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */

    /*
     * code=255, hex=0xFF, ascii="!^"
     */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
    0x00,  /* 00000 */
};

// draws a raster font character on canvas
// only supports 5x8 and 6x8 fonts ATM
void Segment::drawCharacter(unsigned char chr, int16_t x, int16_t y, uint8_t w, uint8_t h, uint32_t color) {
  const uint16_t cols = virtualWidth();
  const uint16_t rows = virtualHeight();

  if (w<5 || w>6 || h!=8) return;
  for (uint8_t i = 0; i<h; i++) { // character height
    int16_t y0 = y + i;
    if (y0 < 0) continue; // drawing off-screen
    if (y0 >= rows) break; // drawing off-screen
    uint8_t bits = 0;
    switch (w) {
      case 5: bits = pgm_read_byte_near(&console_font_5x8[(chr * 8) + i]); break;
      case 6: bits = pgm_read_byte_near(&console_font_6x8[(chr * 8) + i]); break;
    }
    for (uint8_t j = 0; j<w; j++) { // character width
      int16_t x0 = x + (w-1) - j;
      if ((x0 >= 0 || x0 < cols) && ((bits>>(j+(8-w))) & 0x01)) { // bit set & drawing on-screen
        setPixelColorXY(x0, y0, color);
      }
    }
  }
}

#define WU_WEIGHT(a,b) ((uint8_t) (((a)*(b)+(a)+(b))>>8))
void Segment::wu_pixel(uint32_t x, uint32_t y, CRGB c) {      //awesome wu_pixel procedure by reddit u/sutaburosu
  // extract the fractional parts and derive their inverses
  uint8_t xx = x & 0xff, yy = y & 0xff, ix = 255 - xx, iy = 255 - yy;
  // calculate the intensities for each affected pixel
  uint8_t wu[4] = {WU_WEIGHT(ix, iy), WU_WEIGHT(xx, iy),
                   WU_WEIGHT(ix, yy), WU_WEIGHT(xx, yy)};
  // multiply the intensities by the colour, and saturating-add them to the pixels
  for (uint8_t i = 0; i < 4; i++) {
    CRGB led = getPixelColorXY((x >> 8) + (i & 1), (y >> 8) + ((i >> 1) & 1));
    led.r = qadd8(led.r, c.r * wu[i] >> 8);
    led.g = qadd8(led.g, c.g * wu[i] >> 8);
    led.b = qadd8(led.b, c.b * wu[i] >> 8);
    setPixelColorXY(int((x >> 8) + (i & 1)), int((y >> 8) + ((i >> 1) & 1)), led);
  }
}
#undef WU_WEIGHT

#endif // WLED_DISABLE_2D
