/*!
 *  Based on Adafruit's Dotstar library.  C and STM32 compatible.
 */

#include "dotstar.h"
#include "string.h"
#include "stdlib.h"

typedef struct {
  SPI_HandleTypeDef* spiHandle;            ///< SPI Handle
  uint16_t          numLEDs;               ///< Number of pixels
  uint8_t           brightness;            ///< Global brightness setting
  uint8_t          *pixels;                ///< LED RGB values (3 bytes ea.)
  uint8_t           rOffset;               ///< Index of red in 3-byte pixel
  uint8_t           gOffset;               ///< Index of green byte
  uint8_t           bOffset;               ///< Index of blue byte
} DotStar_State;


DotStar_State dotstar_state;

void Dotstar_Init(DotStar_InitHandle* dotstar_init){

   dotstar_state.spiHandle = dotstar_init->spiHandle;
   dotstar_state.numLEDs = dotstar_init->numLEDs;
   dotstar_state.brightness = 0;
   dotstar_state.rOffset = dotstar_init->colorOrder & 3;
   dotstar_state.gOffset = (dotstar_init->colorOrder >> 2) & 3;
   dotstar_state.bOffset = (dotstar_init->colorOrder >> 4) & 3;

   uint16_t bytes = (dotstar_state.rOffset == dotstar_state.gOffset) ?
     dotstar_state.numLEDs + ((dotstar_state.numLEDs + 3) / 4) : // MONO: 10 bits/pixel, round up to next byte
     dotstar_state.numLEDs * 3;              // COLOR: 3 bytes/pixel

   dotstar_state.pixels = (uint8_t *)malloc(bytes);
}

#define spi_out(n) HAL_SPI_Transmit(dotstar_state.spiHandle, n, 1, HAL_MAX_DELAY);

/* ISSUE DATA TO LED STRIP -------------------------------------------------

  Although the LED driver has an additional per-pixel 5-bit brightness
  setting, it is NOT used or supported here. On APA102, the normally
  very fast PWM is gated through a much slower PWM (about 400 Hz),
  rendering it useless for POV or other high-speed things that are
  probably why one is using DotStars instead of NeoPixels in the first
  place. I'm told that some APA102 clones use current control rather than
  PWM for this, which would be much more worthwhile. Still, no support
  here, no plans for it. If you really can't live without it, you can fork
  the library and add it for your own use, but any pull requests for this
  are unlikely be merged for the foreseeable future.
*/

/*!
  @brief   Transmit pixel data in RAM to DotStars.
*/
void ds_show(void) {

  if(!dotstar_state.pixels) return;

  uint8_t *ptr = dotstar_state.pixels, i;            // -> LED data
  uint16_t n   = dotstar_state.numLEDs;              // Counter
  uint16_t b16 = (uint16_t)dotstar_state.brightness; // Type-convert for fixed-point math
  uint8_t sendval;

    sendval = 0x00;
    for(i=0; i<4; i++) spi_out(&sendval);    // 4 byte start-frame marker

    if(dotstar_state.brightness) {                     // Scale pixel brightness on output
      do {                               // For each pixel...
    	sendval = 0xFF;
        spi_out(&sendval);                   //  Pixel start
        for(i=0; i<3; i++) {
        	sendval = (*ptr++ * b16) >> 8;
        	spi_out(&sendval); // Scale, write RGB
        }
      } while(--n);
    } else {                             // Full brightness (no scaling)
      do {                               // For each pixel...
    	sendval = 0xFF;
        spi_out(&sendval);                   //  Pixel start
        for(i=0; i<3; i++) {
        	sendval = *ptr++;
        	spi_out(&sendval); // Write R,G,B
        }
      } while(--n);
    }
    // Four end-frame bytes are seemingly indistinguishable from a white
    // pixel, and empirical testing suggests it can be left out...but it's
    // always a good idea to follow the datasheet, in case future hardware
    // revisions are more strict (e.g. might mandate use of end-frame
    // before start-frame marker). i.e. let's not remove this. But after
    // testing a bit more the suggestion is to use at least (numLeds+1)/2
    // high values (1) or (numLeds+15)/16 full bytes as EndFrame. For details see also:
    // https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/
    sendval = 0xFF;
    for(i=0; i<((dotstar_state.numLEDs + 15) / 16); i++) spi_out(&sendval);

}

/*!
  @brief   Fill the whole DotStar strip with 0 / black / off.
*/
void ds_clear() {
  memset(dotstar_state.pixels, 0, (dotstar_state.rOffset == dotstar_state.gOffset) ?
    dotstar_state.numLEDs + ((dotstar_state.numLEDs + 3) / 4) : // MONO: 10 bits/pixel
    dotstar_state.numLEDs * 3);                   // COLOR: 3 bytes/pixel
}

/*!
  @brief   Set a pixel's color using separate red, green and blue components.
  @param   n  Pixel index, starting from 0.
  @param   r  Red brightness, 0 = minimum (off), 255 = maximum.
  @param   g  Green brightness, 0 = minimum (off), 255 = maximum.
  @param   b  Blue brightness, 0 = minimum (off), 255 = maximum.
*/
void ds_setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if(n < dotstar_state.numLEDs) {
    uint8_t *p = &dotstar_state.pixels[n * 3];
    p[dotstar_state.rOffset] = r;
    p[dotstar_state.gOffset] = g;
    p[dotstar_state.bOffset] = b;
  }
}

/*!
  @brief   Set a pixel's color using a 32-bit 'packed' RGB value.
  @param   n  Pixel index, starting from 0.
  @param   c  32-bit color value. Most significant byte is 0, second is
              red, then green, and least significant byte is blue.
              e.g. 0x00RRGGBB
*/
void ds_setPixelColor32B(uint16_t n, uint32_t c) {
  if(n < dotstar_state.numLEDs) {
    uint8_t *p = &dotstar_state.pixels[n * 3];
    p[dotstar_state.rOffset] = (uint8_t)(c >> 16);
    p[dotstar_state.gOffset] = (uint8_t)(c >>  8);
    p[dotstar_state.bOffset] = (uint8_t)c;
  }
}

/*!
  @brief   Fill all or part of the DotStar strip with a color.
  @param   c      32-bit color value. Most significant byte is 0, second
                  is red, then green, and least significant byte is blue.
                  e.g. 0x00RRGGBB. If all arguments are unspecified, this
                  will be 0 (off).
  @param   first  Index of first pixel to fill, starting from 0. Must be
                  in-bounds, no clipping is performed. 0 if unspecified.
  @param   count  Number of pixels to fill, as a positive value. Passing
                  0 or leaving unspecified will fill to end of strip.
*/
void ds_fill(uint32_t c, uint16_t first, uint16_t count) {
  uint16_t i, end;

  if(first >= dotstar_state.numLEDs) {
    return; // If first LED is past end of strip, nothing to do
  }

  // Calculate the index ONE AFTER the last pixel to fill
  if(count == 0) {
    // Fill to end of strip
    end = dotstar_state.numLEDs;
  } else {
    // Ensure that the loop won't go past the last pixel
    end = first + count;
    if(end > dotstar_state.numLEDs) end = dotstar_state.numLEDs;
  }

  for(i = first; i < end; i++) {
    ds_setPixelColor32B(i, c);
  }
}

/*!
  @brief   Convert hue, saturation and value into a packed 32-bit RGB color
           that can be passed to setPixelColor() or other RGB-compatible
           functions.
  @param   hue  An unsigned 16-bit value, 0 to 65535, representing one full
                loop of the color wheel, which allows 16-bit hues to "roll
                over" while still doing the expected thing (and allowing
                more precision than the wheel() function that was common to
                prior DotStar and NeoPixel examples).
  @param   sat  Saturation, 8-bit value, 0 (min or pure grayscale) to 255
                (max or pure hue). Default of 255 if unspecified.
  @param   val  Value (brightness), 8-bit value, 0 (min / black / off) to
                255 (max or full brightness). Default of 255 if unspecified.
  @return  Packed 32-bit RGB color. Result is linearly but not perceptually
           correct, so you may want to pass the result through the gamma32()
           function (or your own gamma-correction operation) else colors may
           appear washed out. This is not done automatically by this
           function because coders may desire a more refined gamma-
           correction function than the simplified one-size-fits-all
           operation of gamma32(). Diffusing the LEDs also really seems to
           help when using low-saturation colors.
*/
uint32_t ColorHSV(uint16_t hue, uint8_t sat, uint8_t val) {

  uint8_t r, g, b;

  // Remap 0-65535 to 0-1529. Pure red is CENTERED on the 64K rollover;
  // 0 is not the start of pure red, but the midpoint...a few values above
  // zero and a few below 65536 all yield pure red (similarly, 32768 is the
  // midpoint, not start, of pure cyan). The 8-bit RGB hexcone (256 values
  // each for red, green, blue) really only allows for 1530 distinct hues
  // (not 1536, more on that below), but the full unsigned 16-bit type was
  // chosen for hue so that one's code can easily handle a contiguous color
  // wheel by allowing hue to roll over in either direction.
  hue = (hue * 1530L + 32768) / 65536;
  // Because red is centered on the rollover point (the +32768 above,
  // essentially a fixed-point +0.5), the above actually yields 0 to 1530,
  // where 0 and 1530 would yield the same thing. Rather than apply a
  // costly modulo operator, 1530 is handled as a special case below.

  // So you'd think that the color "hexcone" (the thing that ramps from
  // pure red, to pure yellow, to pure green and so forth back to red,
  // yielding six slices), and with each color component having 256
  // possible values (0-255), might have 1536 possible items (6*256),
  // but in reality there's 1530. This is because the last element in
  // each 256-element slice is equal to the first element of the next
  // slice, and keeping those in there this would create small
  // discontinuities in the color wheel. So the last element of each
  // slice is dropped...we regard only elements 0-254, with item 255
  // being picked up as element 0 of the next slice. Like this:
  // Red to not-quite-pure-yellow is:        255,   0, 0 to 255, 254,   0
  // Pure yellow to not-quite-pure-green is: 255, 255, 0 to   1, 255,   0
  // Pure green to not-quite-pure-cyan is:     0, 255, 0 to   0, 255, 254
  // and so forth. Hence, 1530 distinct hues (0 to 1529), and hence why
  // the constants below are not the multiples of 256 you might expect.

  // Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
  if(hue < 510) {         // Red to Green-1
    b = 0;
    if(hue < 255) {       //   Red to Yellow-1
      r = 255;
      g = hue;            //     g = 0 to 254
    } else {              //   Yellow to Green-1
      r = 510 - hue;      //     r = 255 to 1
      g = 255;
    }
  } else if(hue < 1020) { // Green to Blue-1
    r = 0;
    if(hue <  765) {      //   Green to Cyan-1
      g = 255;
      b = hue - 510;      //     b = 0 to 254
    } else {              //   Cyan to Blue-1
      g = 1020 - hue;     //     g = 255 to 1
      b = 255;
    }
  } else if(hue < 1530) { // Blue to Red-1
    g = 0;
    if(hue < 1275) {      //   Blue to Magenta-1
      r = hue - 1020;     //     r = 0 to 254
      b = 255;
    } else {              //   Magenta to Red-1
      r = 255;
      b = 1530 - hue;     //     b = 255 to 1
    }
  } else {                // Last 0.5 Red (quicker than % operator)
    r = 255;
    g = b = 0;
  }

  // Apply saturation and value to R,G,B, pack into 32-bit result:
  uint32_t v1 =   1 + val; // 1 to 256; allows >>8 instead of /255
  uint16_t s1 =   1 + sat; // 1 to 256; same reason
  uint8_t  s2 = 255 - sat; // 255 to 0
  return ((((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8) |
          (((((g * s1) >> 8) + s2) * v1) & 0xff00)       |
         ( ((((b * s1) >> 8) + s2) * v1)           >> 8);
}

/*!
  @brief   Query the color of a previously-set pixel.
  @param   n  Index of pixel to read (0 = first).
  @return  'Packed' 32-bit RGB value. Most significant byte is 0, second is
           is red, then green, and least significant byte is blue.
*/
uint32_t ds_getPixelColor(uint16_t n){
  if(n >= dotstar_state.numLEDs) return 0;
  uint8_t *p = &dotstar_state.pixels[n * 3];
  return ((uint32_t)p[dotstar_state.rOffset] << 16) |
         ((uint32_t)p[dotstar_state.gOffset] <<  8) |
          (uint32_t)p[dotstar_state.bOffset];
}

/*!
  @brief   Adjust output brightness. Does not immediately affect what's
           currently displayed on the LEDs. The next call to show() will
           refresh the LEDs at this level.
  @param   b  Brightness setting, 0=minimum (off), 255=brightest.
  @note    For various reasons I think brightness is better handled in
           one's sketch, but it's here for parity with the NeoPixel
           library. Good news is that brightness setting in this library
           is 'non destructive' -- it's applied as color data is being
           issued to the strip, not during setPixelColor(), and also
           means that getPixelColor() returns the exact value originally
           stored.
*/
void ds_setBrightness(uint8_t b) {
  // Stored brightness value is different than what's passed. This
  // optimizes the actual scaling math later, allowing a fast 8x8-bit
  // multiply and taking the MSB. 'brightness' is a uint8_t, adding 1
  // here may (intentionally) roll over...so 0 = max brightness (color
  // values are interpreted literally; no scaling), 1 = min brightness
  // (off), 255 = just below max brightness.
  dotstar_state.brightness = b + 1;
}

/*!
  @brief   Retrieve the last-set brightness value for the strip.
  @return  Brightness value: 0 = minimum (off), 255 = maximum.
*/
uint8_t ds_getBrightness(void){
  return dotstar_state.brightness - 1; // Reverse above operation
}

/*!
  @brief   A gamma-correction function for 32-bit packed RGB colors.
           Makes color transitions appear more perceptially correct.
  @param   x  32-bit packed RGB color.
  @return  Gamma-adjusted packed color, can then be passed in one of the
           setPixelColor() functions. Like gamma8(), this uses a fixed
           gamma correction exponent of 2.6, which seems reasonably okay
           for average DotStars in average tasks. If you need finer
           control you'll need to provide your own gamma-correction
           function instead.
*/
uint32_t gamma32(uint32_t x) {
  uint8_t *y = (uint8_t *)&x;
  // All four bytes of a 32-bit value are filtered to avoid a bunch of
  // shifting and masking that would be necessary for properly handling
  // different endianisms (and each byte is a fairly trivial operation,
  // so it might not even be wasting cycles vs a check and branch.
  // In theory this might cause trouble *if* someone's storing information
  // in the unused most significant byte of an RGB value, but this seems
  // exceedingly rare and if it's encountered in reality they can mask
  // values going in or coming out.
  for(uint8_t i=0; i<4; i++) y[i] = gamma8(y[i]);
  return x; // Packed 32-bit return
}


uint8_t          *ds_getPixels(void) {
  return dotstar_state.pixels;
}

uint16_t          ds_numPixels(void) {
  return dotstar_state.numLEDs;
}
