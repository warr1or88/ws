//this code is a modified version of https://github.com/Makuna/NeoPixelBus/issues/103
#ifndef NpbWrapper_h
#define NpbWrapper_h

//PIN CONFIGURATION
#ifndef LEDPIN
#define LEDPIN 2  // Legacy pin def required by some other portions of code. This pin is not used do drive LEDs.
#endif
//#define WLED_USE_ANALOG_LEDS //Uncomment for using "dumb" PWM controlled LEDs (see pins below, default R: gpio5, G: 12, B: 15, W: 13)
//#define WLED_USE_H801 //H801 controller. Please uncomment #define WLED_USE_ANALOG_LEDS as well
//#define WLED_USE_5CH_LEDS  //5 Channel H801 for cold and warm white
//#define WLED_USE_BWLT11
//#define WLED_USE_SHOJO_PCB
//END CONFIGURATION

#ifdef WLED_USE_ANALOG_LEDS
  //PWM pins - PINs 15,13,12,14 (W2 = 04)are used with H801 Wifi LED Controller
  #ifdef WLED_USE_H801
    #define RPIN 15   //R pin for analog LED strip   
    #define GPIN 13   //G pin for analog LED strip
    #define BPIN 12   //B pin for analog LED strip
    #define WPIN 14   //W pin for analog LED strip 
    #define W2PIN 04  //W2 pin for analog LED strip
    #undef BTNPIN
    #undef IRPIN
    #define IRPIN  0 //infrared pin (-1 to disable)  MagicHome: 4, H801 Wifi: 0
  #elif defined(WLED_USE_BWLT11)
  //PWM pins - to use with BW-LT11
    #define RPIN 12  //R pin for analog LED strip
    #define GPIN 4   //G pin for analog LED strip
    #define BPIN 14  //B pin for analog LED strip
    #define WPIN 5   //W pin for analog LED strip
  #elif defined(WLED_USE_SHOJO_PCB)
  //PWM pins - to use with Shojo PCB (https://www.bastelbunker.de/esp-rgbww-wifi-led-controller-vbs-edition/)
    #define RPIN 14  //R pin for analog LED strip
    #define GPIN 4   //G pin for analog LED strip
    #define BPIN 5   //B pin for analog LED strip
    #define WPIN 15  //W pin for analog LED strip
    #define W2PIN 12 //W2 pin for analog LED strip
  #elif defined(WLED_USE_PLJAKOBS_PCB)
  // PWM pins - to use with esp_rgbww_controller from patrickjahns/pljakobs (https://github.com/pljakobs/esp_rgbww_controller)
    #define RPIN 12  //R pin for analog LED strip
    #define GPIN 13  //G pin for analog LED strip
    #define BPIN 14  //B pin for analog LED strip
    #define WPIN 4   //W pin for analog LED strip
    #define W2PIN 5  //W2 pin for analog LED strip
    #undef IRPIN
  #else
  //Enable override of Pins by using the platformio_override.ini file
  //PWM pins - PINs 5,12,13,15 are used with Magic Home LED Controller
    #ifndef RPIN
      #define RPIN 5   //R pin for analog LED strip
    #endif
    #ifndef GPIN
      #define GPIN 12  //G pin for analog LED strip
    #endif
    #ifndef BPIN
      #define BPIN 15  //B pin for analog LED strip
    #endif
    #ifndef WPIN
      #define WPIN 13  //W pin for analog LED strip
    #endif
  #endif
  #undef RLYPIN
  #define RLYPIN -1 //disable as pin 12 is used by analog LEDs
#endif

//automatically uses the right driver method for each platform
#ifdef ARDUINO_ARCH_ESP32
  #define PIXELMETHOD NeoEsp32Rmt0Ws2812xMethod
  #define NeoPixelBrightnessBusGrb0  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt0Ws2812xMethod>
  #define NeoPixelBrightnessBusGrb1  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt1Ws2812xMethod>
  #define NeoPixelBrightnessBusGrb2  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt2Ws2812xMethod>
  #define NeoPixelBrightnessBusGrb3  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt3Ws2812xMethod>
  #define NeoPixelBrightnessBusGrb4  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt4Ws2812xMethod>
  #define NeoPixelBrightnessBusGrb5  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt5Ws2812xMethod>
  #define NeoPixelBrightnessBusGrb6  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt6Ws2812xMethod>
  #define NeoPixelBrightnessBusGrb7  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp32Rmt7Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw0 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt0Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw1 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt1Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw2 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt2Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw3 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt3Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw4 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt4Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw5 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt5Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw6 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt6Ws2812xMethod>
  #define NeoPixelBrightnessBusGrbw7 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp32Rmt7Ws2812xMethod>
#else //esp8266
  #define PIXELMETHOD NeoEsp8266BitBang800KbpsMethod
  #define NeoPixelBrightnessBusGrb0  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp8266BitBang800KbpsMethod>
  #define NeoPixelBrightnessBusGrb1  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp8266Uart0Ws2813Method>
  #define NeoPixelBrightnessBusGrb2  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp8266Uart1Ws2813Method>
  #define NeoPixelBrightnessBusGrb3  NeoPixelBrightnessBus<NeoGrbFeature,NeoEsp8266Dma800KbpsMethod>
  #define NeoPixelBrightnessBusGrbw0 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp8266BitBang800KbpsMethod>
  #define NeoPixelBrightnessBusGrbw1 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp8266Uart0Ws2813Method>
  #define NeoPixelBrightnessBusGrbw2 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp8266Uart1Ws2813Method>
  #define NeoPixelBrightnessBusGrbw3 NeoPixelBrightnessBus<NeoGrbwFeature,NeoEsp8266Dma800KbpsMethod>
#endif
#define NeoPixelBrightnessBusWS2801  NeoPixelBrightnessBus<NeoRbgFeature,NeoWs2801Method>
#define NeoPixelBrightnessBusAPA102  NeoPixelBrightnessBus<DotStarBgrFeature,DotStarMethod>
#define NeoPixelBrightnessBusAPA102W NeoPixelBrightnessBus<DotStarLbgrFeature,DotStarMethod>
#define NeoPixelBrightnessBusLPD8806 NeoPixelBrightnessBus<Lpd8806GrbFeature,Lpd8806Method>
#define NeoPixelBrightnessBusTM1814  NeoPixelBrightnessBus<NeoWrgbTm1814Feature,NeoTm1814Method>
#define NeoPixelBrightnessBusP9813   NeoPixelBrightnessBus<P9813BgrFeature,P9813Method>


#include <NeoPixelBrightnessBus.h>
#include "const.h"

#define IS_STRIP_REVERSED(s) ((bool)((pixelStripReversed >> s) & 0x01))
/*
 * #define STRIP_REVERSE_MODE(s,m) (m ? pixelStripReversed |= (0x0001 << s) : pixelStripReversed &= ~(0x0001 << s))
 */

enum NeoPixelType
{
  NeoPixelType_None = 0,
  NeoPixelType_Grb  = 1,
  NeoPixelType_Grbw = 2,
  NeoPixelType_End  = 3
};

class NeoPixelWrapper
{
public:
  NeoPixelWrapper() :
    _type(NeoPixelType_None),
    pixelStrips(0),
    pixelStripReversed(0)
  {
    for (uint8_t i=0; i < MAX_NUMBER_OF_STRIPS; i++)
    {
      _pGRB[i] = NULL;
      pixelColorOrder[i]=0;
    }
  }

  ~NeoPixelWrapper()
  {
    cleanup();
  }

  void initStrips(uint8_t numStrips, int8_t stripPins[][2], uint16_t stripLen[], uint8_t ledType[], uint8_t colorOrder[], uint8_t skipFirst, uint16_t reverseMode)
  {
    cleanup();

    uint16_t totalPixels = 0;
    pixelStrips = numStrips;
    pixelStripReversed = reverseMode; // bit mapped info
    pixelSkipAmount = skipFirst;
    for (uint8_t idx = 0; idx < numStrips; idx++)
    {
      pixelType[idx]          = ledType[idx];
      pixelCounts[idx]        = stripLen[idx] + skipFirst;
      pixelStripPins[idx]     = stripPins[idx][0];
      pixelStripPinsClk[idx]  = stripPins[idx][1];
      pixelStripStartIdx[idx] = totalPixels;
      pixelColorOrder[idx]    = colorOrder[idx];
      totalPixels            += pixelCounts[idx];
    }
  }

  void Begin(NeoPixelType type)
  {
    cleanup();

    _type = type;
    for (uint8_t idx = 0; idx < pixelStrips; idx++)
    {
      switch (pixelType[idx]) {

        case TYPE_SK6812_RGBW: //RGBW bit unset
        case TYPE_WS2812_RGB:  //RGBW bit unset
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb0(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->Begin(); break;
            case 1:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb1(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->Begin(); break;
            case 2:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb2(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->Begin(); break;
            case 3:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb3(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->Begin(); break;
            case 4:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb4(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb4*)_pGRB[idx])->Begin(); break;
            case 5:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb5(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb5*)_pGRB[idx])->Begin(); break;
            case 6:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb6(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb6*)_pGRB[idx])->Begin(); break;
            case 7:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb7(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb7*)_pGRB[idx])->Begin(); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb1(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->Begin(); break;
            case 2:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb2(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->Begin(); break;
            case 3:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb3(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->Begin(); break;
            default:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrb0(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->Begin(); break;
          }
          #endif
        }
        break;

        case TYPE_WS2812_RGB | 0x80:  //RGBW bit set
        case TYPE_SK6812_RGBW | 0x80: //RGBW bit set
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw0(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->Begin(); break;
            case 1:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw1(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->Begin(); break;
            case 2:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw2(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->Begin(); break;
            case 3:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw3(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->Begin(); break;
            case 4:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw4(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw4*)_pGRB[idx])->Begin(); break;
            case 5:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw5(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw5*)_pGRB[idx])->Begin(); break;
            case 6:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw6(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw6*)_pGRB[idx])->Begin(); break;
            case 7:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw7(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw7*)_pGRB[idx])->Begin(); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw1(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->Begin(); break;
            case 2:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw2(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->Begin(); break;
            case 3:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw3(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->Begin(); break;
            default:
              _pGRB[idx] = (void*) new NeoPixelBrightnessBusGrbw0(pixelCounts[idx], pixelStripPins[idx]); ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->Begin(); break;
          }
          #endif
        }
        break;

        case TYPE_WS2801:  //RGBW bit unset
        case TYPE_WS2801 | 0x80:  //RGBW bit set
          _pGRB[idx] = (void*) new NeoPixelBrightnessBusWS2801(pixelCounts[idx], pixelStripPinsClk[idx], pixelStripPins[idx]);
          ((NeoPixelBrightnessBusWS2801*)_pGRB[idx])->Begin();
        break;

        case TYPE_APA102:  //RGBW bit unset
            _pGRB[idx] = (void*) new NeoPixelBrightnessBusAPA102(pixelCounts[idx], pixelStripPinsClk[idx], pixelStripPins[idx]);
            ((NeoPixelBrightnessBusAPA102*)_pGRB[idx])->Begin();
        break;

        case TYPE_APA102 | 0x80:  //RGBW bit set
            _pGRB[idx] = (void*) new NeoPixelBrightnessBusAPA102W(pixelCounts[idx], pixelStripPinsClk[idx], pixelStripPins[idx]);
            ((NeoPixelBrightnessBusAPA102W*)_pGRB[idx])->Begin();
        break;

        case TYPE_LPD8806: //RGBW bit unset
        case TYPE_LPD8806 | 0x80: //RGBW bit set
          _pGRB[idx] = (void*) new NeoPixelBrightnessBusLPD8806(pixelCounts[idx], pixelStripPinsClk[idx], pixelStripPins[idx]);
          ((NeoPixelBrightnessBusLPD8806*)_pGRB[idx])->Begin();
        break;

        case TYPE_P9813:   //RGBW bit unset
        case TYPE_P9813 | 0x80:   //RGBW bit set
          _pGRB[idx] = (void*) new NeoPixelBrightnessBusP9813(pixelCounts[idx], pixelStripPinsClk[idx], pixelStripPins[idx]);
          ((NeoPixelBrightnessBusP9813*)_pGRB[idx])->Begin();
        break;

        case TYPE_TM1814:  //RGBW bit unset
        case TYPE_TM1814 | 0x80:  //RGBW bit set
          _pGRB[idx] = (void*) new NeoPixelBrightnessBusTM1814(pixelCounts[idx], pixelStripPins[idx]);
          ((NeoPixelBrightnessBusTM1814*)_pGRB[idx])->Begin();
        break;

      }

      // clear potential sacrificial pixels (they may never get updated again)
      //SetPixelColorRaw(pixelStripStartIdx[idx], RgbwColor(0,0,0,0));
    }

    #ifdef WLED_USE_ANALOG_LEDS 
      #ifdef ARDUINO_ARCH_ESP32
        ledcSetup(0, 5000, 8);
        ledcAttachPin(RPIN, 0);
        ledcSetup(1, 5000, 8);
        ledcAttachPin(GPIN, 1);
        ledcSetup(2, 5000, 8);        
        ledcAttachPin(BPIN, 2);
        if(_type == NeoPixelType_Grbw) 
        {
          ledcSetup(3, 5000, 8);        
          ledcAttachPin(WPIN, 3);
          #ifdef WLED_USE_5CH_LEDS
            ledcSetup(4, 5000, 8);        
            ledcAttachPin(W2PIN, 4);
          #endif
        }
      #else  // ESP8266
        //init PWM pins
        pinMode(RPIN, OUTPUT);
        pinMode(GPIN, OUTPUT);
        pinMode(BPIN, OUTPUT); 
        if(_type == NeoPixelType_Grbw) 
        {
          pinMode(WPIN, OUTPUT); 
          #ifdef WLED_USE_5CH_LEDS
            pinMode(W2PIN, OUTPUT);
          #endif
        }
        analogWriteRange(255);  //same range as one RGB channel
        analogWriteFreq(880);   //PWM frequency proven as good for LEDs
      #endif 
    #endif
  }

#ifdef WLED_USE_ANALOG_LEDS      
  void SetRgbwPwm(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2=0)
  {
    #ifdef ARDUINO_ARCH_ESP32
      ledcWrite(0, r);
      ledcWrite(1, g);
      ledcWrite(2, b);
      switch (_type) {
        case NeoPixelType_Grb:                                                  break;
        #ifdef WLED_USE_5CH_LEDS
          case NeoPixelType_Grbw: ledcWrite(3, w); ledcWrite(4, w2);            break;
        #else
          case NeoPixelType_Grbw: ledcWrite(3, w);                              break;
        #endif
      }        
    #else   // ESP8266
      analogWrite(RPIN, r);
      analogWrite(GPIN, g);
      analogWrite(BPIN, b);
      switch (_type) {
        case NeoPixelType_Grb:                                                  break;
        #ifdef WLED_USE_5CH_LEDS
          case NeoPixelType_Grbw: analogWrite(WPIN, w); analogWrite(W2PIN, w2); break;
        #else
          case NeoPixelType_Grbw: analogWrite(WPIN, w);                         break;
        #endif
      }
    #endif 
  }
#endif

  void Show()
  {
    for (uint8_t idx = 0; idx < pixelStrips; idx++)
    {
      switch (pixelType[idx]) {

        case TYPE_SK6812_RGBW: //RGBW bit unset
        case TYPE_WS2812_RGB:  //RGBW bit unset
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->Show(); break;
            case 1: ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->Show(); break;
            case 2: ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->Show(); break;
            case 3: ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->Show(); break;
            case 4: ((NeoPixelBrightnessBusGrb4*)_pGRB[idx])->Show(); break;
            case 5: ((NeoPixelBrightnessBusGrb5*)_pGRB[idx])->Show(); break;
            case 6: ((NeoPixelBrightnessBusGrb6*)_pGRB[idx])->Show(); break;
            case 7: ((NeoPixelBrightnessBusGrb7*)_pGRB[idx])->Show(); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->Show(); break;
            case 2: ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->Show(); break;
            case 3: ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->Show(); break;
            default: ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->Show(); break;
          }
          #endif
        }
        break;

        case TYPE_WS2812_RGB | 0x80:  //RGBW bit set
        case TYPE_SK6812_RGBW | 0x80: //RGBW bit set
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->Show(); break;
            case 1: ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->Show(); break;
            case 2: ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->Show(); break;
            case 3: ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->Show(); break;
            case 4: ((NeoPixelBrightnessBusGrbw4*)_pGRB[idx])->Show(); break;
            case 5: ((NeoPixelBrightnessBusGrbw5*)_pGRB[idx])->Show(); break;
            case 6: ((NeoPixelBrightnessBusGrbw6*)_pGRB[idx])->Show(); break;
            case 7: ((NeoPixelBrightnessBusGrbw7*)_pGRB[idx])->Show(); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->Show(); break;
            case 2: ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->Show(); break;
            case 3: ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->Show(); break;
            default: ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->Show(); break;
          }
          #endif
        }
        break;

        case TYPE_WS2801:
        case TYPE_WS2801 | 0x80:  //RGBW bit set
          ((NeoPixelBrightnessBusWS2801*)_pGRB[idx])->Show();
        break;

        case TYPE_APA102:
            ((NeoPixelBrightnessBusAPA102*)_pGRB[idx])->Show();
        break;

        case TYPE_APA102 | 0x80:  //RGBW bit set
            ((NeoPixelBrightnessBusAPA102W*)_pGRB[idx])->Show();
        break;

        case TYPE_LPD8806:
        case TYPE_LPD8806 | 0x80: //RGBW bit set
          ((NeoPixelBrightnessBusLPD8806*)_pGRB[idx])->Show();
        break;

        case TYPE_P9813:
        case TYPE_P9813 | 0x80:   //RGBW bit set
          ((NeoPixelBrightnessBusP9813*)_pGRB[idx])->Show();
        break;

        case TYPE_TM1814:
        case TYPE_TM1814 | 0x80:  //RGBW bit set
          ((NeoPixelBrightnessBusTM1814*)_pGRB[idx])->Show();
        break;

      }
    }
  }

  bool CanShow()
  {
    bool canShow = true;
    for (uint8_t idx = 0; idx < pixelStrips; idx++)
    {
      if (!_pGRB[idx]) continue;  // prevent uninitialised panic
      switch (pixelType[idx]) {

        case TYPE_SK6812_RGBW: //RGBW bit unset
        case TYPE_WS2812_RGB:  //RGBW bit unset
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: canShow &= ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->CanShow(); break;
            case 1: canShow &= ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->CanShow(); break;
            case 2: canShow &= ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->CanShow(); break;
            case 3: canShow &= ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->CanShow(); break;
            case 4: canShow &= ((NeoPixelBrightnessBusGrb4*)_pGRB[idx])->CanShow(); break;
            case 5: canShow &= ((NeoPixelBrightnessBusGrb5*)_pGRB[idx])->CanShow(); break;
            case 6: canShow &= ((NeoPixelBrightnessBusGrb6*)_pGRB[idx])->CanShow(); break;
            case 7: canShow &= ((NeoPixelBrightnessBusGrb7*)_pGRB[idx])->CanShow(); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: canShow &= ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->CanShow(); break;
            case 2: canShow &= ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->CanShow(); break;
            case 3: canShow &= ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->CanShow(); break;
            default: canShow &= ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->CanShow(); break;
          }
          #endif
        }
        break;

        case TYPE_WS2812_RGB | 0x80:  //RGBW bit set
        case TYPE_SK6812_RGBW | 0x80: //RGBW bit set
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: canShow &= ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->CanShow(); break;
            case 1: canShow &= ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->CanShow(); break;
            case 2: canShow &= ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->CanShow(); break;
            case 3: canShow &= ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->CanShow(); break;
            case 4: canShow &= ((NeoPixelBrightnessBusGrbw4*)_pGRB[idx])->CanShow(); break;
            case 5: canShow &= ((NeoPixelBrightnessBusGrbw5*)_pGRB[idx])->CanShow(); break;
            case 6: canShow &= ((NeoPixelBrightnessBusGrbw6*)_pGRB[idx])->CanShow(); break;
            case 7: canShow &= ((NeoPixelBrightnessBusGrbw7*)_pGRB[idx])->CanShow(); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: canShow &= ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->CanShow(); break;
            case 2: canShow &= ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->CanShow(); break;
            case 3: canShow &= ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->CanShow(); break;
            default: canShow &= ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->CanShow(); break;
          }
          #endif
        }
        break;

        case TYPE_WS2801:
        case TYPE_WS2801 | 0x80:  //RGBW bit set
          canShow &= ((NeoPixelBrightnessBusWS2801*)_pGRB[idx])->CanShow();
        break;

        case TYPE_APA102:
            canShow &= ((NeoPixelBrightnessBusAPA102*)_pGRB[idx])->CanShow();
        break;

        case TYPE_APA102 | 0x80:  //RGBW bit set
            canShow &= ((NeoPixelBrightnessBusAPA102W*)_pGRB[idx])->CanShow();
        break;

        case TYPE_LPD8806:
        case TYPE_LPD8806 | 0x80: //RGBW bit set
          canShow &= ((NeoPixelBrightnessBusLPD8806*)_pGRB[idx])->CanShow();
        break;

        case TYPE_P9813:
        case TYPE_P9813 | 0x80:   //RGBW bit set
          canShow &= ((NeoPixelBrightnessBusP9813*)_pGRB[idx])->CanShow();
        break;

        case TYPE_TM1814:
        case TYPE_TM1814 | 0x80:  //RGBW bit set
          canShow &= ((NeoPixelBrightnessBusTM1814*)_pGRB[idx])->CanShow();
        break;

      }
    }
    return canShow;
  }

  void SetPixelColorRaw(uint16_t indexPixel, RgbwColor c)
  {
    // figure out which strip this pixel index is on
    uint8_t stripIdx = GetStripFromRealPixel(indexPixel);

    // subtract strip start index so we're addressing just this strip instead of all pixels on all strips
    indexPixel -= pixelStripStartIdx[stripIdx];
	  // pixelCounts contains sacrificial pixel
    if (IS_STRIP_REVERSED(stripIdx)) indexPixel = pixelCounts[stripIdx] - 1 - indexPixel + pixelSkipAmount;

    RgbColor rgb = RgbColor(c.R, c.G, c.B);

    switch (pixelType[stripIdx]) {

      case TYPE_SK6812_RGBW: //RGBW bit unset
      case TYPE_WS2812_RGB:  //RGBW bit unset
      {
        #ifdef ARDUINO_ARCH_ESP32
        switch (stripIdx)
        {
          case 0: ((NeoPixelBrightnessBusGrb0*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 1: ((NeoPixelBrightnessBusGrb1*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 2: ((NeoPixelBrightnessBusGrb2*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 3: ((NeoPixelBrightnessBusGrb3*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 4: ((NeoPixelBrightnessBusGrb4*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 5: ((NeoPixelBrightnessBusGrb5*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 6: ((NeoPixelBrightnessBusGrb6*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 7: ((NeoPixelBrightnessBusGrb7*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
        }
        #else
        switch (pixelStripPins[stripIdx])
        {
          case 1: ((NeoPixelBrightnessBusGrb1*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 2: ((NeoPixelBrightnessBusGrb2*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          case 3: ((NeoPixelBrightnessBusGrb3*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
          default: ((NeoPixelBrightnessBusGrb0*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb); break;
        }
        #endif
      }
      break;

      case TYPE_WS2812_RGB | 0x80:  //RGBW bit set
      case TYPE_SK6812_RGBW | 0x80: //RGBW bit set
      {
        #ifdef ARDUINO_ARCH_ESP32
        switch (stripIdx)
        {
          case 0: ((NeoPixelBrightnessBusGrbw0*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 1: ((NeoPixelBrightnessBusGrbw1*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 2: ((NeoPixelBrightnessBusGrbw2*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 3: ((NeoPixelBrightnessBusGrbw3*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 4: ((NeoPixelBrightnessBusGrbw4*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 5: ((NeoPixelBrightnessBusGrbw5*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 6: ((NeoPixelBrightnessBusGrbw6*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 7: ((NeoPixelBrightnessBusGrbw7*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
        }
        #else
        switch (pixelStripPins[stripIdx])
        {
          case 1: ((NeoPixelBrightnessBusGrbw1*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 2: ((NeoPixelBrightnessBusGrbw2*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          case 3: ((NeoPixelBrightnessBusGrbw3*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
          default: ((NeoPixelBrightnessBusGrbw0*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c); break;
        }
        #endif
      }
      break;

      case TYPE_WS2801:
      case TYPE_WS2801 | 0x80:  //RGBW bit set
        ((NeoPixelBrightnessBusWS2801*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb);
      break;

      case TYPE_APA102:
          ((NeoPixelBrightnessBusAPA102*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb);
      break;

      case TYPE_APA102 | 0x80:  //RGBW bit set
          ((NeoPixelBrightnessBusAPA102W*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c);
      break;

      case TYPE_LPD8806:
      case TYPE_LPD8806 | 0x80: //RGBW bit set
        ((NeoPixelBrightnessBusLPD8806*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb);
      break;

      case TYPE_P9813:
      case TYPE_P9813 | 0x80:   //RGBW bit set
        ((NeoPixelBrightnessBusP9813*)_pGRB[stripIdx])->SetPixelColor(indexPixel, rgb);
      break;

      case TYPE_TM1814:
      case TYPE_TM1814 | 0x80:  //RGBW bit set
        ((NeoPixelBrightnessBusTM1814*)_pGRB[stripIdx])->SetPixelColor(indexPixel, c);
      break;

    }
  }

  void SetPixelColor(uint16_t indexPixel, RgbwColor c)
  {
    /*
    Set pixel color with necessary color order conversion.
    */

    RgbwColor col;

    // take into account sacrificial pixels
    indexPixel = GetRealPixelIndex(indexPixel);

    uint8_t co = pixelColorOrder[GetStripFromRealPixel(indexPixel)];

    //reorder channels to selected order
    switch (co)
    {
      case  0: col.G = c.G; col.R = c.R; col.B = c.B; break; //0 = GRB, default
      case  1: col.G = c.R; col.R = c.G; col.B = c.B; break; //1 = RGB, common for WS2811
      case  2: col.G = c.B; col.R = c.R; col.B = c.G; break; //2 = BRG
      case  3: col.G = c.R; col.R = c.B; col.B = c.G; break; //3 = RBG
      case  4: col.G = c.B; col.R = c.G; col.B = c.R; break; //4 = BGR
      default: col.G = c.G; col.R = c.B; col.B = c.R; break; //5 = GBR
    }
    col.W = c.W;

    SetPixelColorRaw(indexPixel, col);
  }

  void SetBrightness(byte b)
  {
    for (uint8_t idx = 0; idx < pixelStrips; idx++)
    {
      switch (pixelType[idx]) {

        case TYPE_SK6812_RGBW: //RGBW bit unset
        case TYPE_WS2812_RGB:  //RGBW bit unset
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->SetBrightness(b); break;
            case 1: ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->SetBrightness(b); break;
            case 2: ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->SetBrightness(b); break;
            case 3: ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->SetBrightness(b); break;
            case 4: ((NeoPixelBrightnessBusGrb4*)_pGRB[idx])->SetBrightness(b); break;
            case 5: ((NeoPixelBrightnessBusGrb5*)_pGRB[idx])->SetBrightness(b); break;
            case 6: ((NeoPixelBrightnessBusGrb6*)_pGRB[idx])->SetBrightness(b); break;
            case 7: ((NeoPixelBrightnessBusGrb7*)_pGRB[idx])->SetBrightness(b); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: ((NeoPixelBrightnessBusGrb1*)_pGRB[idx])->SetBrightness(b); break;
            case 2: ((NeoPixelBrightnessBusGrb2*)_pGRB[idx])->SetBrightness(b); break;
            case 3: ((NeoPixelBrightnessBusGrb3*)_pGRB[idx])->SetBrightness(b); break;
            default: ((NeoPixelBrightnessBusGrb0*)_pGRB[idx])->SetBrightness(b); break;
          }
          #endif
        }
        break;

        case TYPE_WS2812_RGB | 0x80:  //RGBW bit set
        case TYPE_SK6812_RGBW | 0x80: //RGBW bit set
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->SetBrightness(b); break;
            case 1: ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->SetBrightness(b); break;
            case 2: ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->SetBrightness(b); break;
            case 3: ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->SetBrightness(b); break;
            case 4: ((NeoPixelBrightnessBusGrbw4*)_pGRB[idx])->SetBrightness(b); break;
            case 5: ((NeoPixelBrightnessBusGrbw5*)_pGRB[idx])->SetBrightness(b); break;
            case 6: ((NeoPixelBrightnessBusGrbw6*)_pGRB[idx])->SetBrightness(b); break;
            case 7: ((NeoPixelBrightnessBusGrbw7*)_pGRB[idx])->SetBrightness(b); break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx])->SetBrightness(b); break;
            case 2: ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx])->SetBrightness(b); break;
            case 3: ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx])->SetBrightness(b); break;
            default: ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx])->SetBrightness(b); break;
          }
          #endif
        }
        break;

        case TYPE_WS2801:
        case TYPE_WS2801 | 0x80:  //RGBW bit set
          ((NeoPixelBrightnessBusWS2801*)_pGRB[idx])->SetBrightness(b);
        break;

        case TYPE_APA102:
            ((NeoPixelBrightnessBusAPA102*)_pGRB[idx])->SetBrightness(b);
        break;

        case TYPE_APA102 | 0x80:  //RGBW bit set
            ((NeoPixelBrightnessBusAPA102W*)_pGRB[idx])->SetBrightness(b);
        break;

        case TYPE_LPD8806:
        case TYPE_LPD8806 | 0x80: //RGBW bit set
          ((NeoPixelBrightnessBusLPD8806*)_pGRB[idx])->SetBrightness(b);
        break;

        case TYPE_P9813:
        case TYPE_P9813 | 0x80:   //RGBW bit set
          ((NeoPixelBrightnessBusP9813*)_pGRB[idx])->SetBrightness(b);
        break;

        case TYPE_TM1814:
        case TYPE_TM1814 | 0x80:  //RGBW bit set
          ((NeoPixelBrightnessBusTM1814*)_pGRB[idx])->SetBrightness(b);
        break;

      }
    }
  }

  void SetColorOrder(byte colorOrder, uint8_t strip=0)
  {
    if (strip>pixelStrips) return;
    pixelColorOrder[strip] = colorOrder;
  }

  uint8_t GetColorOrder(uint8_t strip=0)
  {
    if (strip>pixelStrips) return 0;
    return pixelColorOrder[strip];
  }

  // get real (physical) pixel color (including sacrificial pixels)
  RgbwColor GetPixelColorRaw(uint16_t indexPixel)
  {
    // take into account sacrificial pixels
    indexPixel = GetRealPixelIndex(indexPixel);

    // figure out which strip this pixel index is on
    uint8_t stripIdx = GetStripFromRealPixel(indexPixel);

    // subtract strip start index so we're addressing just this strip instead of all pixels on all strips
    indexPixel -= pixelStripStartIdx[stripIdx];
	  // pixelCounts contains sacrificial pixel
    if (IS_STRIP_REVERSED(stripIdx)) indexPixel = pixelCounts[stripIdx] - 1 - indexPixel + pixelSkipAmount;

    switch (pixelType[stripIdx]) {

      case TYPE_SK6812_RGBW: //RGBW bit unset
      case TYPE_WS2812_RGB:  //RGBW bit unset
      {
        #ifdef ARDUINO_ARCH_ESP32
        switch (stripIdx)
        {
          case 0: return ((NeoPixelBrightnessBusGrb0*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 1: return ((NeoPixelBrightnessBusGrb1*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 2: return ((NeoPixelBrightnessBusGrb2*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 3: return ((NeoPixelBrightnessBusGrb3*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 4: return ((NeoPixelBrightnessBusGrb4*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 5: return ((NeoPixelBrightnessBusGrb5*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 6: return ((NeoPixelBrightnessBusGrb6*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 7: return ((NeoPixelBrightnessBusGrb7*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
        }
        #else
        switch (pixelStripPins[stripIdx])
        {
          case 1: return ((NeoPixelBrightnessBusGrb1*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 2: return ((NeoPixelBrightnessBusGrb2*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 3: return ((NeoPixelBrightnessBusGrb3*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          default: return ((NeoPixelBrightnessBusGrb0*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
        }
        #endif
      }
      break;

      case TYPE_WS2812_RGB | 0x80:  //RGBW bit set
      case TYPE_SK6812_RGBW | 0x80: //RGBW bit set
      {
        #ifdef ARDUINO_ARCH_ESP32
        switch (stripIdx)
        {
          case 0: return ((NeoPixelBrightnessBusGrbw0*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 1: return ((NeoPixelBrightnessBusGrbw1*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 2: return ((NeoPixelBrightnessBusGrbw2*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 3: return ((NeoPixelBrightnessBusGrbw3*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 4: return ((NeoPixelBrightnessBusGrbw4*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 5: return ((NeoPixelBrightnessBusGrbw5*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 6: return ((NeoPixelBrightnessBusGrbw6*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 7: return ((NeoPixelBrightnessBusGrbw7*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
        }
        #else
        switch (pixelStripPins[stripIdx])
        {
          case 1: return ((NeoPixelBrightnessBusGrbw1*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 2: return ((NeoPixelBrightnessBusGrbw2*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          case 3: return ((NeoPixelBrightnessBusGrbw3*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
          default: return ((NeoPixelBrightnessBusGrbw0*)_pGRB[stripIdx])->GetPixelColor(indexPixel); break;
        }
        #endif
      }
      break;

      case TYPE_WS2801:
      case TYPE_WS2801 | 0x80:  //RGBW bit set
        return ((NeoPixelBrightnessBusWS2801*)_pGRB[stripIdx])->GetPixelColor(indexPixel);
      break;

      case TYPE_APA102:
          return ((NeoPixelBrightnessBusAPA102*)_pGRB[stripIdx])->GetPixelColor(indexPixel);
      break;

      case TYPE_APA102 | 0x80:  //RGBW bit set
          return ((NeoPixelBrightnessBusAPA102W*)_pGRB[stripIdx])->GetPixelColor(indexPixel);
      break;

      case TYPE_LPD8806:
      case TYPE_LPD8806 | 0x80: //RGBW bit set
        return ((NeoPixelBrightnessBusLPD8806*)_pGRB[stripIdx])->GetPixelColor(indexPixel);
      break;

      case TYPE_P9813:
      case TYPE_P9813 | 0x80:   //RGBW bit set
        return ((NeoPixelBrightnessBusP9813*)_pGRB[stripIdx])->GetPixelColor(indexPixel);
      break;

      case TYPE_TM1814:
      case TYPE_TM1814 | 0x80:  //RGBW bit set
        return ((NeoPixelBrightnessBusTM1814*)_pGRB[stripIdx])->GetPixelColor(indexPixel);
      break;

    }
    return 0;
  }

  // NOTE: Due to feature differences, some support RGBW but the method name
  // here needs to be unique, thus GetPixeColorRgbw
  uint32_t GetPixelColorRgbw(uint16_t indexPixel)
  {
    RgbwColor col = GetPixelColorRaw(indexPixel);
    uint8_t co = pixelColorOrder[GetStripFromRealPixel(GetRealPixelIndex(indexPixel))];

    switch (co)
    {
      //                    W               G              R               B
      case  0: return ((col.W << 24) | (col.G << 8) | (col.R << 16) | (col.B)); //0 = GRB, default
      case  1: return ((col.W << 24) | (col.R << 8) | (col.G << 16) | (col.B)); //1 = RGB, common for WS2811
      case  2: return ((col.W << 24) | (col.B << 8) | (col.R << 16) | (col.G)); //2 = BRG
      case  3: return ((col.W << 24) | (col.B << 8) | (col.G << 16) | (col.R)); //3 = RBG
      case  4: return ((col.W << 24) | (col.R << 8) | (col.B << 16) | (col.G)); //4 = BGR
      case  5: return ((col.W << 24) | (col.G << 8) | (col.B << 16) | (col.R)); //5 = GBR
    }

    return 0;
  }

  bool isReversed(uint8_t strip) {
    if (strip >= pixelStrips) return 0;
    return IS_STRIP_REVERSED(strip);
  }
  
private:
  NeoPixelType _type;
  byte      pixelColorOrder[MAX_NUMBER_OF_STRIPS];
  uint8_t   pixelStrips;                              // number of strips
  uint8_t   pixelType[MAX_NUMBER_OF_STRIPS];          // LED pixel type
  uint16_t  pixelCounts[MAX_NUMBER_OF_STRIPS];        // number of pixels on each strip
  int8_t    pixelStripPins[MAX_NUMBER_OF_STRIPS];     // strip GPIO pin
  int8_t    pixelStripPinsClk[MAX_NUMBER_OF_STRIPS];  // strip GPIO pin
  uint16_t  pixelStripStartIdx[MAX_NUMBER_OF_STRIPS]; // start index in a single virtual strip
  uint16_t  pixelStripReversed;                       // bit mapped info if strip is reversed (max 16 strips)
  uint8_t   pixelSkipAmount;                          // sacrificial (skipped) pixel count (same for each string)

  void *_pGRB[MAX_NUMBER_OF_STRIPS];

  // extracts strip index from real (physical) pixel index (including sacrificial pixels)
  uint8_t GetStripFromRealPixel(uint16_t indexPixel)
  {
    // figure out which strip this (real) pixel is on
    uint8_t stripIdx = 0;
    for (uint8_t idx = 0; idx < pixelStrips; idx++)
      if (indexPixel >= pixelStripStartIdx[idx])
        stripIdx = idx;
      else
        break;
    return stripIdx;
  }

  // calculates real (physical) pixel index from logical index (adding sacrificial pixel for each strip)
  uint16_t GetRealPixelIndex(uint16_t indexPixel)
  {
    if (!pixelSkipAmount) return indexPixel;
    indexPixel += pixelSkipAmount;
    for (uint8_t idx = 1; idx < pixelStrips; idx++) {
      if (indexPixel < pixelStripStartIdx[idx]) break;
      indexPixel += pixelSkipAmount;  // each strip has its own sacrificial pixel
    }
    return indexPixel;
  }

  void cleanup()
  {
    while (!CanShow()) yield(); // wait while pixels are updating
    for (uint8_t idx = 0; idx < pixelStrips; idx++)
    {
      switch (pixelType[idx]) {

        case TYPE_SK6812_RGBW: //RGBW bit unset
        case TYPE_WS2812_RGB:  //RGBW bit unset
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: delete ((NeoPixelBrightnessBusGrb0*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 1: delete ((NeoPixelBrightnessBusGrb1*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 2: delete ((NeoPixelBrightnessBusGrb2*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 3: delete ((NeoPixelBrightnessBusGrb3*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 4: delete ((NeoPixelBrightnessBusGrb4*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 5: delete ((NeoPixelBrightnessBusGrb5*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 6: delete ((NeoPixelBrightnessBusGrb6*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 7: delete ((NeoPixelBrightnessBusGrb7*)_pGRB[idx]); _pGRB[idx] = NULL; break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: delete ((NeoPixelBrightnessBusGrb1*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 2: delete ((NeoPixelBrightnessBusGrb2*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 3: delete ((NeoPixelBrightnessBusGrb3*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            default: delete ((NeoPixelBrightnessBusGrb0*)_pGRB[idx]); _pGRB[idx] = NULL; break;
          }
          #endif
        }
        break;

        case TYPE_WS2812_RGB | 0x80:  //RGBW bit set
        case TYPE_SK6812_RGBW | 0x80: //RGBW bit set
        {
          #ifdef ARDUINO_ARCH_ESP32
          switch (idx)
          {
            case 0: delete ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 1: delete ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 2: delete ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 3: delete ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 4: delete ((NeoPixelBrightnessBusGrbw4*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 5: delete ((NeoPixelBrightnessBusGrbw5*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 6: delete ((NeoPixelBrightnessBusGrbw6*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 7: delete ((NeoPixelBrightnessBusGrbw7*)_pGRB[idx]); _pGRB[idx] = NULL; break;
          }
          #else
          switch (pixelStripPins[idx])
          {
            case 1: delete ((NeoPixelBrightnessBusGrbw1*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 2: delete ((NeoPixelBrightnessBusGrbw2*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            case 3: delete ((NeoPixelBrightnessBusGrbw3*)_pGRB[idx]); _pGRB[idx] = NULL; break;
            default: delete ((NeoPixelBrightnessBusGrbw0*)_pGRB[idx]); _pGRB[idx] = NULL; break;
          }
          #endif
        }
        break;

        case TYPE_WS2801:
        case TYPE_WS2801 | 0x80:  //RGBW bit set
          delete ((NeoPixelBrightnessBusWS2801*)_pGRB[idx]); _pGRB[idx] = NULL;
        break;

        case TYPE_APA102:
            delete ((NeoPixelBrightnessBusAPA102*)_pGRB[idx]); _pGRB[idx] = NULL;
        break;

        case TYPE_APA102 | 0x80:  //RGBW bit set
            delete ((NeoPixelBrightnessBusAPA102W*)_pGRB[idx]); _pGRB[idx] = NULL;
        break;

        case TYPE_LPD8806:
        case TYPE_LPD8806 | 0x80: //RGBW bit set
          delete ((NeoPixelBrightnessBusLPD8806*)_pGRB[idx]); _pGRB[idx] = NULL;
        break;

        case TYPE_P9813:
        case TYPE_P9813 | 0x80:   //RGBW bit set
          delete ((NeoPixelBrightnessBusP9813*)_pGRB[idx]); _pGRB[idx] = NULL;
        break;

        case TYPE_TM1814:
        case TYPE_TM1814 | 0x80:  //RGBW bit set
          delete ((NeoPixelBrightnessBusTM1814*)_pGRB[idx]); _pGRB[idx] = NULL;
        break;

      }
    }
  }
};
#endif
