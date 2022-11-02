#include "wled.h"

#include "palettes.h"

/*
 * JSON API (De)serialization
 */

void deserializeSegment(JsonObject elem, byte it, byte presetId)
{
  byte id = elem["id"] | it;
  if (id >= strip.getMaxSegments()) return;

  int stop = elem["stop"] | -1;

  // if using vectors use this code to append segment
  if (id >= strip.getSegmentsNum()) {
    if (stop <= 0) return; // ignore empty/inactive segments
    strip.appendSegment(Segment(0, strip.getLengthTotal()));
    id = strip.getSegmentsNum()-1; // segments are added at the end of list
  }

  Segment& seg = strip.getSegment(id);
  Segment prev = seg; //make a backup so we can tell if something changed

  uint16_t start = elem["start"] | seg.start;
  if (stop < 0) {
    uint16_t len = elem["len"];
    stop = (len > 0) ? start + len : seg.stop;
  }
  // 2D segments
  uint16_t startY = elem["startY"] | seg.startY;
  uint16_t stopY = elem["stopY"] | seg.stopY;

  //repeat, multiplies segment until all LEDs are used, or max segments reached
  bool repeat = elem["rpt"] | false;
  if (repeat && stop>0) {
    elem.remove("id");  // remove for recursive call
    elem.remove("rpt"); // remove for recursive call
    elem.remove("n");   // remove for recursive call
    uint16_t len = stop - start;
    for (size_t i=id+1; i<strip.getMaxSegments(); i++) {
      start = start + len;
      if (start >= strip.getLengthTotal()) break;
      //TODO: add support for 2D
      elem["start"] = start;
      elem["stop"]  = start + len;
      elem["rev"]   = !elem["rev"]; // alternate reverse on even/odd segments
      deserializeSegment(elem, i, presetId); // recursive call with new id
    }
    return;
  }

  if (elem["n"]) {
    // name field exists
    if (seg.name) { //clear old name
      delete[] seg.name;
      seg.name = nullptr;
    }

    const char * name = elem["n"].as<const char*>();
    size_t len = 0;
    if (name != nullptr) len = strlen(name);
    if (len > 0 && len < 33) {
      seg.name = new char[len+1];
      if (seg.name) strlcpy(seg.name, name, 33);
    } else {
      // but is empty (already deleted above)
      elem.remove("n");
    }
  } else if (start != seg.start || stop != seg.stop) {
    // clearing or setting segment without name field
    if (seg.name) {
      delete[] seg.name;
      seg.name = nullptr;
    }
  }

  uint16_t grp = elem["grp"] | seg.grouping;
  uint16_t spc = elem[F("spc")] | seg.spacing;
  uint16_t of  = seg.offset;
  uint8_t  soundSim = elem[F("ssim")] | seg.soundSim;
  uint8_t  map1D2D  = elem[F("mp12")] | seg.map1D2D;

  if ((spc>0 && spc!=seg.spacing) || seg.map1D2D!=map1D2D) seg.fill(BLACK); // clear spacing gaps

  seg.map1D2D  = map1D2D & 0x07;
  seg.soundSim = soundSim & 0x07;

  uint16_t len = 1;
  if (stop > start) len = stop - start;
  int offset = elem[F("of")] | INT32_MAX;
  if (offset != INT32_MAX) {
    int offsetAbs = abs(offset);
    if (offsetAbs > len - 1) offsetAbs %= len;
    if (offset < 0) offsetAbs = len - offsetAbs;
    of = offsetAbs;
  }
  if (stop > start && of > len -1) of = len -1;
  strip.setSegment(id, start, stop, grp, spc, of, startY, stopY);

  byte segbri = seg.opacity;
  if (getVal(elem["bri"], &segbri)) {
    if (segbri > 0) seg.setOpacity(segbri);
    seg.setOption(SEG_OPTION_ON, segbri);
  }

  bool on = elem["on"] | seg.getOption(SEG_OPTION_ON);
  if (elem["on"].is<const char*>() && elem["on"].as<const char*>()[0] == 't') on = !on;
  seg.setOption(SEG_OPTION_ON, on);
  bool frz = elem["frz"] | seg.getOption(SEG_OPTION_FREEZE);
  if (elem["frz"].is<const char*>() && elem["frz"].as<const char*>()[0] == 't') frz = !seg.getOption(SEG_OPTION_FREEZE);
  seg.setOption(SEG_OPTION_FREEZE, frz);

  seg.setCCT(elem["cct"] | seg.cct);

  JsonArray colarr = elem["col"];
  if (!colarr.isNull())
  {
    for (size_t i = 0; i < 3; i++)
    {
      int rgbw[] = {0,0,0,0};
      bool colValid = false;
      JsonArray colX = colarr[i];
      if (colX.isNull()) {
        byte brgbw[] = {0,0,0,0};
        const char* hexCol = colarr[i];
        if (hexCol == nullptr) { //Kelvin color temperature (or invalid), e.g 2400
          int kelvin = colarr[i] | -1;
          if (kelvin <  0) continue;
          if (kelvin == 0) seg.setColor(i, 0);
          if (kelvin >  0) colorKtoRGB(kelvin, brgbw);
          colValid = true;
        } else { //HEX string, e.g. "FFAA00"
          colValid = colorFromHexString(brgbw, hexCol);
        }
        for (size_t c = 0; c < 4; c++) rgbw[c] = brgbw[c];
      } else { //Array of ints (RGB or RGBW color), e.g. [255,160,0]
        byte sz = colX.size();
        if (sz == 0) continue; //do nothing on empty array

        copyArray(colX, rgbw, 4);
        colValid = true;
      }

      if (!colValid) continue;

      seg.setColor(i, RGBW32(rgbw[0],rgbw[1],rgbw[2],rgbw[3]));
      if (seg.mode == FX_MODE_STATIC) strip.trigger(); //instant refresh
    }
  }

  // lx parser
  #ifdef WLED_ENABLE_LOXONE
  int lx = elem[F("lx")] | -1;
  if (lx > 0) {
    parseLxJson(lx, id, false);
  }
  int ly = elem[F("ly")] | -1;
  if (ly > 0) {
    parseLxJson(ly, id, true);
  }
  #endif

  seg.setOption(SEG_OPTION_SELECTED, elem["sel"]     | seg.getOption(SEG_OPTION_SELECTED));
  seg.setOption(SEG_OPTION_REVERSED, elem["rev"]     | seg.getOption(SEG_OPTION_REVERSED));
  seg.setOption(SEG_OPTION_MIRROR  , elem[F("mi")]   | seg.getOption(SEG_OPTION_MIRROR  ));
  #ifndef WLED_DISABLE_2D
  seg.setOption(SEG_OPTION_REVERSED_Y, elem[F("rY")] | seg.getOption(SEG_OPTION_REVERSED_Y));
  seg.setOption(SEG_OPTION_MIRROR_Y  , elem[F("mY")] | seg.getOption(SEG_OPTION_MIRROR_Y  ));
  seg.setOption(SEG_OPTION_TRANSPOSED, elem[F("tp")] | seg.getOption(SEG_OPTION_TRANSPOSED));
  #endif

  byte fx = seg.mode;
  if (getVal(elem["fx"], &fx, 0, strip.getModeCount())) { //load effect ('r' random, '~' inc/dec, 0-255 exact value)
    if (!presetId && currentPlaylist>=0) unloadPlaylist();
    if (fx != seg.mode) {
      seg.startTransition(strip.getTransition()); // set effect transitions
      //seg.markForReset();
      seg.mode = fx;
      // load default values from effect string if effect is selected without
      // any other effect parameter (i.e. effect clicked in UI)
      if ( elem[F("sx")].isNull()
        && elem[F("ix")].isNull()
        && elem["pal"].isNull()
        && elem[F("c1")].isNull()
        && elem[F("c2")].isNull()
        && elem[F("c3")].isNull() )
      {
        int16_t sOpt;
        sOpt = extractModeDefaults(fx, SET_F("sx"));   if (sOpt >= 0) seg.speed     = sOpt;
        sOpt = extractModeDefaults(fx, SET_F("ix"));   if (sOpt >= 0) seg.intensity = sOpt;
        sOpt = extractModeDefaults(fx, SET_F("c1"));   if (sOpt >= 0) seg.custom1   = sOpt;
        sOpt = extractModeDefaults(fx, SET_F("c2"));   if (sOpt >= 0) seg.custom2   = sOpt;
        sOpt = extractModeDefaults(fx, SET_F("c3"));   if (sOpt >= 0) seg.custom3   = sOpt;
        sOpt = extractModeDefaults(fx, SET_F("mp12")); if (sOpt >= 0) seg.map1D2D   = sOpt & 0x07;
        sOpt = extractModeDefaults(fx, SET_F("ssim")); if (sOpt >= 0) seg.soundSim  = sOpt & 0x07;
        sOpt = extractModeDefaults(fx, "rev");         if (sOpt >= 0) seg.reverse   = (bool)sOpt;
        sOpt = extractModeDefaults(fx, SET_F("mi"));   if (sOpt >= 0) seg.mirror    = (bool)sOpt; // NOTE: setting this option is a risky business
        sOpt = extractModeDefaults(fx, SET_F("rY"));   if (sOpt >= 0) seg.reverse_y = (bool)sOpt;
        sOpt = extractModeDefaults(fx, SET_F("mY"));   if (sOpt >= 0) seg.mirror_y  = (bool)sOpt; // NOTE: setting this option is a risky business
        sOpt = extractModeDefaults(fx, "pal");
        if (sOpt >= 0 && sOpt < strip.getPaletteCount() + strip.customPalettes.size()) {
          if (sOpt != seg.palette) {
            if (strip.paletteFade && !seg.transitional) seg.startTransition(strip.getTransition());
            seg.palette = sOpt;
          }
        }
      }
    }
  }

  //getVal also supports inc/decrementing and random
  getVal(elem[F("sx")], &seg.speed);
  getVal(elem[F("ix")], &seg.intensity);
  uint8_t pal = seg.palette;
  if (getVal(elem["pal"], &pal, 1, strip.getPaletteCount())) {
    if (pal != seg.palette) {
      if (strip.paletteFade && !seg.transitional) seg.startTransition(strip.getTransition());
      seg.palette = pal;
    }
  }
  getVal(elem[F("c1")], &seg.custom1);
  getVal(elem[F("c2")], &seg.custom2);
  uint8_t cust3 = seg.custom3;
  getVal(elem[F("c3")], &cust3); // we can't pass reference to bifield
  seg.custom3 = cust3;

  seg.check1 = elem[F("o1")] | seg.check1;
  seg.check2 = elem[F("o2")] | seg.check2;
  seg.check3 = elem[F("o3")] | seg.check3;
  
  JsonArray iarr = elem[F("i")]; //set individual LEDs
  if (!iarr.isNull()) {
    // set brightness immediately and disable transition
    transitionDelayTemp = 0;
    jsonTransitionOnce = true;
    strip.setBrightness(scaledBri(bri), true);

    // freeze and init to black
    if (!seg.getOption(SEG_OPTION_FREEZE)) {
      seg.setOption(SEG_OPTION_FREEZE, true);
      seg.fill(BLACK);
    }

    uint16_t start = 0, stop = 0;
    byte set = 0; //0 nothing set, 1 start set, 2 range set

    for (size_t i = 0; i < iarr.size(); i++) {
      if(iarr[i].is<JsonInteger>()) {
        if (!set) {
          start = iarr[i];
          set = 1;
        } else {
          stop = iarr[i];
          set = 2;
        }
      } else { //color
        uint8_t rgbw[] = {0,0,0,0};
        JsonArray icol = iarr[i];
        if (!icol.isNull()) { //array, e.g. [255,0,0]
          byte sz = icol.size();
          if (sz > 0 && sz < 5) copyArray(icol, rgbw);
        } else { //hex string, e.g. "FF0000"
          byte brgbw[] = {0,0,0,0};
          const char* hexCol = iarr[i];
          if (colorFromHexString(brgbw, hexCol)) {
            for (size_t c = 0; c < 4; c++) rgbw[c] = brgbw[c];
          }
        }

        if (set < 2) stop = start + 1;
        for (int i = start; i < stop; i++) {
          if (strip.gammaCorrectCol) {
            seg.setPixelColor(i, gamma8(rgbw[0]), gamma8(rgbw[1]), gamma8(rgbw[2]), gamma8(rgbw[3]));
          } else {
            seg.setPixelColor(i, rgbw[0], rgbw[1], rgbw[2], rgbw[3]);
          }
        }
        if (!set) start++;
        set = 0;
      }
    }
    strip.trigger();
  }
  // send UDP if not in preset and something changed that is not just selection
  // send UDP if something changed that is not just selection or segment power/opacity
  if ((seg.differs(prev) & 0x7E) && seg.getOption(SEG_OPTION_ON)==prev.getOption(SEG_OPTION_ON)) stateChanged = true;
}

// deserializes WLED state (fileDoc points to doc object if called from web server)
// presetId is non-0 if called from handlePreset()
bool deserializeState(JsonObject root, byte callMode, byte presetId)
{
  bool stateResponse = root[F("v")] | false;

  bool onBefore = bri;
  uint8_t tmpBri = bri;
  getVal(root["bri"], &tmpBri);

  if (root["on"].isNull()) {
    if ((onBefore && tmpBri==0) || (!onBefore && tmpBri>0)) toggleOnOff();
    bri = tmpBri;
  } else {
    bool on = root["on"] | onBefore;
    if (on != onBefore || (root["on"].is<const char*>() && root["on"].as<const char*>()[0] == 't')) {
      toggleOnOff();
      // a hack is needed after toggleOnOf()
      if (!root["bri"].isNull()) {
        if (bri==0) briLast = tmpBri;
        else        bri     = tmpBri;
      }
    }
  }

  if (bri && !onBefore) { // unfreeze all segments when turning on
    for (size_t s=0; s < strip.getSegmentsNum(); s++) {
      strip.getSegment(s).setOption(SEG_OPTION_FREEZE, false);
    }
    if (realtimeMode && !realtimeOverride && useMainSegmentOnly) { // keep live segment frozen if live
      strip.getMainSegment().setOption(SEG_OPTION_FREEZE, true);
    }
  }

  int tr = -1;
  if (!presetId || currentPlaylist < 0) { //do not apply transition time from preset if playlist active, as it would override playlist transition times
    tr = root[F("transition")] | -1;
    if (tr >= 0)
    {
      transitionDelay = tr;
      transitionDelay *= 100;
      transitionDelayTemp = transitionDelay;
    }
  }

  // temporary transition (applies only once)
  tr = root[F("tt")] | -1;
  if (tr >= 0)
  {
    transitionDelayTemp = tr;
    transitionDelayTemp *= 100;
    jsonTransitionOnce = true;
  }
  strip.setTransition(transitionDelayTemp); // required here for color transitions to have correct duration

  tr = root[F("tb")] | -1;
  if (tr >= 0) strip.timebase = ((uint32_t)tr) - millis();

  JsonObject nl       = root["nl"];
  nightlightActive    = nl["on"]      | nightlightActive;
  nightlightDelayMins = nl["dur"]     | nightlightDelayMins;
  nightlightMode      = nl["mode"]    | nightlightMode;
  nightlightTargetBri = nl[F("tbri")] | nightlightTargetBri;

  JsonObject udpn      = root["udpn"];
  notifyDirect         = udpn["send"] | notifyDirect;
  receiveNotifications = udpn["recv"] | receiveNotifications;
  if ((bool)udpn[F("nn")]) callMode = CALL_MODE_NO_NOTIFY; //send no notification just for this request

  unsigned long timein = root[F("time")] | UINT32_MAX; //backup time source if NTP not synced
  if (timein != UINT32_MAX) {
    setTimeFromAPI(timein);
    if (presetsModifiedTime == 0) presetsModifiedTime = timein;
  }

  doReboot = root[F("rb")] | doReboot;

  // do not allow changing main segment while in realtime mode (may get odd results else)
  if (!realtimeMode) strip.setMainSegmentId(root[F("mainseg")] | strip.getMainSegmentId()); // must be before realtimeLock() if "live"

  realtimeOverride = root[F("lor")] | realtimeOverride;
  if (realtimeOverride > 2) realtimeOverride = REALTIME_OVERRIDE_ALWAYS;
  if (realtimeMode && useMainSegmentOnly) {
    strip.getMainSegment().setOption(SEG_OPTION_FREEZE, !realtimeOverride);
  }

  if (root.containsKey("live")) {
    if (root["live"].as<bool>()) {
      transitionDelayTemp = 0;
      jsonTransitionOnce = true;
      realtimeLock(65000);
    } else {
      exitRealtime();
    }
  }

  int it = 0;
  JsonVariant segVar = root["seg"];
  if (segVar.is<JsonObject>())
  {
    int id = segVar["id"] | -1;
    //if "seg" is not an array and ID not specified, apply to all selected/checked segments
    if (id < 0) {
      //apply all selected segments
      //bool didSet = false;
      for (size_t s = 0; s < strip.getSegmentsNum(); s++) {
        Segment &sg = strip.getSegment(s);
        if (sg.isSelected()) {
          deserializeSegment(segVar, s, presetId);
          //didSet = true;
        }
      }
      //TODO: not sure if it is good idea to change first active but unselected segment
      //if (!didSet) deserializeSegment(segVar, strip.getMainSegmentId(), presetId);
    } else {
      deserializeSegment(segVar, id, presetId); //apply only the segment with the specified ID
    }
  } else {
    JsonArray segs = segVar.as<JsonArray>();
    for (JsonObject elem : segs) {
      deserializeSegment(elem, it, presetId);
      it++;
    }
  }

  usermods.readFromJsonState(root);

  loadLedmap = root[F("ledmap")] | loadLedmap;

  byte ps = root[F("psave")];
  if (ps > 0 && ps < 251) savePreset(ps, nullptr, root);

  ps = root[F("pdel")]; //deletion
  if (ps > 0 && ps < 251) deletePreset(ps);

  // HTTP API commands (must be handled before "ps")
  const char* httpwin = root["win"];
  if (httpwin) {
    String apireq = "win"; apireq += '&'; // reduce flash string usage
    apireq += httpwin;
    handleSet(nullptr, apireq, false);    // may set stateChanged
  }

  // applying preset (2 cases: a) API call includes all preset values, b) API only specifies preset ID)
  if (!root["ps"].isNull()) {
    ps = presetCycCurr;
    if (stateChanged) {
      // a) already applied preset content (requires "seg" or "win" but will ignore the rest)
      currentPreset = root["ps"] | currentPreset;
      // if preset contains HTTP API call do not change presetCycCurr
      if (root["win"].isNull()) presetCycCurr = currentPreset;
      stateChanged = false; // cancel state change update (preset was set directly by applying values stored in UI JSON array)
    } else if (root["win"].isNull() && getVal(root["ps"], &ps, 0, 0) && ps > 0 && ps < 251 && ps != currentPreset) {
      // b) preset ID only (use embedded cycling limits if they exist in getVal())
      presetCycCurr = ps;
      applyPreset(ps, callMode); // async load
      return stateResponse;
    }
  }

  JsonObject playlist = root[F("playlist")];
  if (!playlist.isNull() && loadPlaylist(playlist, presetId)) {
    //do not notify here, because the first playlist entry will do
    if (root["on"].isNull()) callMode = CALL_MODE_NO_NOTIFY;
    else callMode = CALL_MODE_DIRECT_CHANGE;  // possible bugfix for playlist only containing HTTP API preset FX=~
  }

  stateUpdated(callMode);

  return stateResponse;
}

void serializeSegment(JsonObject& root, Segment& seg, byte id, bool forPreset, bool segmentBounds)
{
  root["id"] = id;
  if (segmentBounds) {
    root["start"] = seg.start;
    root["stop"] = seg.stop;
    if (strip.isMatrix) {
      root[F("startY")] = seg.startY;
      root[F("stopY")]  = seg.stopY;
    }
  }
  if (!forPreset) root["len"] = seg.stop - seg.start;
  root["grp"] = seg.grouping;
  root[F("spc")] = seg.spacing;
  root[F("of")] = seg.offset;
  root["on"] = seg.getOption(SEG_OPTION_ON);
  root["frz"] = seg.getOption(SEG_OPTION_FREEZE);
  byte segbri = seg.opacity;
  root["bri"] = (segbri) ? segbri : 255;
  root["cct"] = seg.cct;

  if (segmentBounds && seg.name != nullptr) root["n"] = reinterpret_cast<const char *>(seg.name); //not good practice, but decreases required JSON buffer

  // to conserve RAM we will serialize the col array manually
  // this will reduce RAM footprint from ~300 bytes to 84 bytes per segment
  char colstr[70]; colstr[0] = '['; colstr[1] = '\0';  //max len 68 (5 chan, all 255)
  const char *format = strip.hasWhiteChannel() ? PSTR("[%u,%u,%u,%u]") : PSTR("[%u,%u,%u]");
  for (size_t i = 0; i < 3; i++)
  {
    byte segcol[4]; byte* c = segcol;
    segcol[0] = R(seg.colors[i]);
    segcol[1] = G(seg.colors[i]);
    segcol[2] = B(seg.colors[i]);
    segcol[3] = W(seg.colors[i]);
    char tmpcol[22];
    sprintf_P(tmpcol, format, (unsigned)c[0], (unsigned)c[1], (unsigned)c[2], (unsigned)c[3]);
    strcat(colstr, i<2 ? strcat(tmpcol, ",") : tmpcol);
  }
  strcat(colstr, "]");
  root["col"] = serialized(colstr);

  root["fx"]     = seg.mode;
  root[F("sx")]  = seg.speed;
  root[F("ix")]  = seg.intensity;
  root["pal"]    = seg.palette;
  root[F("c1")]  = seg.custom1;
  root[F("c2")]  = seg.custom2;
  root[F("c3")]  = seg.custom3;
  root[F("sel")] = seg.isSelected();
  root["rev"]    = seg.getOption(SEG_OPTION_REVERSED);
  root[F("mi")]  = seg.getOption(SEG_OPTION_MIRROR);
  if (strip.isMatrix) {
    root[F("rY")] = seg.getOption(SEG_OPTION_REVERSED_Y);
    root[F("mY")] = seg.getOption(SEG_OPTION_MIRROR_Y);
    root[F("tp")] = seg.getOption(SEG_OPTION_TRANSPOSED);
  }
  root[F("o1")]  = seg.check1;
  root[F("o2")]  = seg.check2;
  root[F("o3")]  = seg.check3;
  root[F("ssim")]  = seg.soundSim;
  root[F("mp12")]  = seg.map1D2D;
}

void serializeState(JsonObject root, bool forPreset, bool includeBri, bool segmentBounds)
{
  if (includeBri) {
    root["on"] = (bri > 0);
    root["bri"] = briLast;
    root[F("transition")] = transitionDelay/100; //in 100ms
  }

  if (!forPreset) {
    if (errorFlag) {root[F("error")] = errorFlag; errorFlag = ERR_NONE;} //prevent error message to persist on screen

    root["ps"] = (currentPreset > 0) ? currentPreset : -1;
    root[F("pl")] = currentPlaylist;

    usermods.addToJsonState(root);

    JsonObject nl = root.createNestedObject("nl");
    nl["on"] = nightlightActive;
    nl["dur"] = nightlightDelayMins;
    nl["mode"] = nightlightMode;
    nl[F("tbri")] = nightlightTargetBri;
    if (nightlightActive) {
      nl[F("rem")] = (nightlightDelayMs - (millis() - nightlightStartTime)) / 1000; // seconds remaining
    } else {
      nl[F("rem")] = -1;
    }

    JsonObject udpn = root.createNestedObject("udpn");
    udpn["send"] = notifyDirect;
    udpn["recv"] = receiveNotifications;

    root[F("lor")] = realtimeOverride;
  }

  root[F("mainseg")] = strip.getMainSegmentId();

  bool selectedSegmentsOnly = root[F("sc")] | false;
  JsonArray seg = root.createNestedArray("seg");
  for (size_t s = 0; s < strip.getMaxSegments(); s++) {
    if (s >= strip.getSegmentsNum()) {
      if (forPreset && segmentBounds) { //disable segments not part of preset
        JsonObject seg0 = seg.createNestedObject();
        seg0["stop"] = 0;
        continue;
      } else
        break;
    }
    Segment &sg = strip.getSegment(s);
    if (!forPreset && selectedSegmentsOnly && !sg.isSelected()) continue;
    if (sg.isActive()) {
      JsonObject seg0 = seg.createNestedObject();
      serializeSegment(seg0, sg, s, forPreset, segmentBounds);
    } else if (forPreset && segmentBounds) { //disable segments not part of preset
      JsonObject seg0 = seg.createNestedObject();
      seg0["stop"] = 0;
    }
  }
}

void serializeInfo(JsonObject root)
{
  root[F("ver")] = versionString;
  root[F("vid")] = VERSION;
  //root[F("cn")] = WLED_CODENAME;

  JsonObject leds = root.createNestedObject("leds");
  leds[F("count")] = strip.getLengthTotal();
  leds[F("pwr")] = strip.currentMilliamps;
  leds["fps"] = strip.getFps();
  leds[F("maxpwr")] = (strip.currentMilliamps)? strip.ablMilliampsMax : 0;
  leds[F("maxseg")] = strip.getMaxSegments();
  //leds[F("actseg")] = strip.getActiveSegmentsNum();
  //leds[F("seglock")] = false; //might be used in the future to prevent modifications to segment config
  leds[F("cpal")] = strip.customPalettes.size(); //number of custom palettes

  #ifndef WLED_DISABLE_2D
  if (strip.isMatrix) {
    JsonObject matrix = leds.createNestedObject("matrix");
    matrix["w"] = strip.matrixWidth;
    matrix["h"] = strip.matrixHeight;
  }
  #endif

  uint8_t totalLC = 0;
  JsonArray lcarr = leds.createNestedArray(F("seglc"));
  size_t nSegs = strip.getSegmentsNum();
  for (size_t s = 0; s < nSegs; s++) {
    if (!strip.getSegment(s).isActive()) continue;
    uint8_t lc = strip.getSegment(s).getLightCapabilities();
    totalLC |= lc;
    lcarr.add(lc);
  }

  leds["lc"] = totalLC;

  leds[F("rgbw")] = strip.hasRGBWBus(); // deprecated, use info.leds.lc
  leds[F("wv")]   = totalLC & 0x02;     // deprecated, true if white slider should be displayed for any segment
  leds["cct"]     = totalLC & 0x04;     // deprecated, use info.leds.lc

  #ifdef WLED_DISABLE_AUDIO
  root[F("noaudio")] = true;
  #endif

  #ifdef WLED_DEBUG
  JsonArray i2c = root.createNestedArray(F("i2c"));
  i2c.add(i2c_sda);
  i2c.add(i2c_scl);
  JsonArray spi = root.createNestedArray(F("spi"));
  spi.add(spi_mosi);
  spi.add(spi_sclk);
  spi.add(spi_cs);
  #endif

  root[F("str")] = syncToggleReceive;

  root[F("name")] = serverDescription;
  root[F("udpport")] = udpPort;
  root["live"] = (bool)realtimeMode;
  root[F("liveseg")] = useMainSegmentOnly ? strip.getMainSegmentId() : -1;  // if using main segment only for live

  switch (realtimeMode) {
    case REALTIME_MODE_INACTIVE: root["lm"] = ""; break;
    case REALTIME_MODE_GENERIC:  root["lm"] = ""; break;
    case REALTIME_MODE_UDP:      root["lm"] = F("UDP"); break;
    case REALTIME_MODE_HYPERION: root["lm"] = F("Hyperion"); break;
    case REALTIME_MODE_E131:     root["lm"] = F("E1.31"); break;
    case REALTIME_MODE_ADALIGHT: root["lm"] = F("USB Adalight/TPM2"); break;
    case REALTIME_MODE_ARTNET:   root["lm"] = F("Art-Net"); break;
    case REALTIME_MODE_TPM2NET:  root["lm"] = F("tpm2.net"); break;
    case REALTIME_MODE_DDP:      root["lm"] = F("DDP"); break;
  }

  if (realtimeIP[0] == 0)
  {
    root[F("lip")] = "";
  } else {
    root[F("lip")] = realtimeIP.toString();
  }

  #ifdef WLED_ENABLE_WEBSOCKETS
  root[F("ws")] = ws.count();
  #else
  root[F("ws")] = -1;
  #endif

  root[F("fxcount")] = strip.getModeCount();
  root[F("palcount")] = strip.getPaletteCount();

  JsonArray ledmaps = root.createNestedArray(F("maps"));
  for (size_t i=0; i<10; i++) {
    if ((ledMaps>>i) & 0x0001) ledmaps.add(i);
  }

  JsonObject wifi_info = root.createNestedObject("wifi");
  wifi_info[F("bssid")] = WiFi.BSSIDstr();
  int qrssi = WiFi.RSSI();
  wifi_info[F("rssi")] = qrssi;
  wifi_info[F("signal")] = getSignalQuality(qrssi);
  wifi_info[F("channel")] = WiFi.channel();

  JsonObject fs_info = root.createNestedObject("fs");
  fs_info["u"] = fsBytesUsed / 1000;
  fs_info["t"] = fsBytesTotal / 1000;
  fs_info[F("pmt")] = presetsModifiedTime;

  root[F("ndc")] = nodeListEnabled ? (int)Nodes.size() : -1;
  
  #ifdef ARDUINO_ARCH_ESP32
  #ifdef WLED_DEBUG
    wifi_info[F("txPower")] = (int) WiFi.getTxPower();
    wifi_info[F("sleep")] = (bool) WiFi.getSleep();
  #endif
  root[F("arch")] = "esp32";
  root[F("core")] = ESP.getSdkVersion();
  //root[F("maxalloc")] = ESP.getMaxAllocHeap();
  #ifdef WLED_DEBUG
    root[F("resetReason0")] = (int)rtc_get_reset_reason(0);
    root[F("resetReason1")] = (int)rtc_get_reset_reason(1);
  #endif
  root[F("lwip")] = 0; //deprecated
  #else
  root[F("arch")] = "esp8266";
  root[F("core")] = ESP.getCoreVersion();
  //root[F("maxalloc")] = ESP.getMaxFreeBlockSize();
  #ifdef WLED_DEBUG
    root[F("resetReason")] = (int)ESP.getResetInfoPtr()->reason;
  #endif
  root[F("lwip")] = LWIP_VERSION_MAJOR;
  #endif

  root[F("freeheap")] = ESP.getFreeHeap();
  #if defined(ARDUINO_ARCH_ESP32) && defined(WLED_USE_PSRAM)
  if (psramFound()) root[F("psram")] = ESP.getFreePsram();
  #endif
  root[F("uptime")] = millis()/1000 + rolloverMillis*4294967;

  usermods.addToJsonInfo(root);

  byte os = 0;
  #ifdef WLED_DEBUG
  os  = 0x80;
  #endif
  #ifndef WLED_DISABLE_ALEXA
  os += 0x40;
  #endif
  #ifndef WLED_DISABLE_BLYNK
  os += 0x20;
  #endif
  #ifdef USERMOD_CRONIXIE
  os += 0x10;
  #endif
  #ifndef WLED_DISABLE_FILESYSTEM
  os += 0x08;
  #endif
  #ifndef WLED_DISABLE_HUESYNC
  os += 0x04;
  #endif
  #ifdef WLED_ENABLE_ADALIGHT
  os += 0x02;
  #endif
  #ifndef WLED_DISABLE_OTA
  os += 0x01;
  #endif
  root[F("opt")] = os;

  root[F("brand")] = "WLED";
  root[F("product")] = F("FOSS");
  root["mac"] = escapedMac;
  char s[16] = "";
  if (Network.isConnected())
  {
    IPAddress localIP = Network.localIP();
    sprintf(s, "%d.%d.%d.%d", localIP[0], localIP[1], localIP[2], localIP[3]);
  }
  root["ip"] = s;
}

void setPaletteColors(JsonArray json, CRGBPalette16 palette)
{
    for (int i = 0; i < 16; i++) {
      JsonArray colors =  json.createNestedArray();
      CRGB color = palette[i];
      colors.add(i<<4);
      colors.add(color.red);
      colors.add(color.green);
      colors.add(color.blue);
    }
}

void setPaletteColors(JsonArray json, byte* tcp)
{
    TRGBGradientPaletteEntryUnion* ent = (TRGBGradientPaletteEntryUnion*)(tcp);
    TRGBGradientPaletteEntryUnion u;

    // Count entries
    uint16_t count = 0;
    do {
        u = *(ent + count);
        count++;
    } while ( u.index != 255);

    u = *ent;
    int indexstart = 0;
    while( indexstart < 255) {
      indexstart = u.index;

      JsonArray colors =  json.createNestedArray();
      colors.add(u.index);
      colors.add(u.r);
      colors.add(u.g);
      colors.add(u.b);

      ent++;
      u = *ent;
    }
}

void serializePalettes(JsonObject root, AsyncWebServerRequest* request)
{
  byte tcp[72];
  #ifdef ESP8266
  int itemPerPage = 5;
  #else
  int itemPerPage = 8;
  #endif

  int page = 0;
  if (request->hasParam("page")) {
    page = request->getParam("page")->value().toInt();
  }

  int palettesCount = strip.getPaletteCount();
  int customPalettes = strip.customPalettes.size();

  int maxPage = (palettesCount + customPalettes -1) / itemPerPage;
  if (page > maxPage) page = maxPage;

  int start = itemPerPage * page;
  int end = start + itemPerPage;
  if (end > palettesCount + customPalettes) end = palettesCount + customPalettes;

  root[F("m")] = maxPage; // inform caller how many pages there are
  JsonObject palettes  = root.createNestedObject("p");

  for (int i = start; i < end; i++) {
    JsonArray curPalette = palettes.createNestedArray(String(i>=palettesCount ? 255 - i + palettesCount : i));
    switch (i) {
      case 0: //default palette
        setPaletteColors(curPalette, PartyColors_p); 
        break;
      case 1: //random
          curPalette.add("r");
          curPalette.add("r");
          curPalette.add("r");
          curPalette.add("r");
        break;
      case 2: //primary color only
        curPalette.add("c1");
        break;
      case 3: //primary + secondary
        curPalette.add("c1");
        curPalette.add("c1");
        curPalette.add("c2");
        curPalette.add("c2");
        break;
      case 4: //primary + secondary + tertiary
        curPalette.add("c3");
        curPalette.add("c2");
        curPalette.add("c1");
        break;
      case 5: //primary + secondary (+tert if not off), more distinct
        curPalette.add("c1");
        curPalette.add("c1");
        curPalette.add("c1");
        curPalette.add("c1");
        curPalette.add("c1");
        curPalette.add("c2");
        curPalette.add("c2");
        curPalette.add("c2");
        curPalette.add("c2");
        curPalette.add("c2");
        curPalette.add("c3");
        curPalette.add("c3");
        curPalette.add("c3");
        curPalette.add("c3");
        curPalette.add("c3");
        curPalette.add("c1");
        break;
      case 6: //Party colors
        setPaletteColors(curPalette, PartyColors_p);
        break;
      case 7: //Cloud colors
        setPaletteColors(curPalette, CloudColors_p);
        break;
      case 8: //Lava colors
        setPaletteColors(curPalette, LavaColors_p);
        break;
      case 9: //Ocean colors
        setPaletteColors(curPalette, OceanColors_p);
        break;
      case 10: //Forest colors
        setPaletteColors(curPalette, ForestColors_p);
        break;
      case 11: //Rainbow colors
        setPaletteColors(curPalette, RainbowColors_p);
        break;
      case 12: //Rainbow stripe colors
        setPaletteColors(curPalette, RainbowStripeColors_p);
        break;
      default:
        {
        if (i>=palettesCount) {
          setPaletteColors(curPalette, strip.customPalettes[i - palettesCount]);
        } else {
          memcpy_P(tcp, (byte*)pgm_read_dword(&(gGradientPalettes[i - 13])), 72);
          setPaletteColors(curPalette, tcp);
        }
        }
        break;
    }
  }
}

void serializeNodes(JsonObject root)
{
  JsonArray nodes = root.createNestedArray("nodes");

  for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
  {
    if (it->second.ip[0] != 0)
    {
      JsonObject node = nodes.createNestedObject();
      node[F("name")] = it->second.nodeName;
      node["type"]    = it->second.nodeType;
      node["ip"]      = it->second.ip.toString();
      node[F("age")]  = it->second.age;
      node[F("vid")]  = it->second.build;
    }
  }
}

void serializeModeData(JsonArray fxdata)
{
  for (size_t i = 0; i < strip.getModeCount(); i++) {
    String lineBuffer = FPSTR(strip.getModeData(i));
    if (lineBuffer.length() > 0) {
      uint8_t endPos = lineBuffer.indexOf('@');
      if (endPos>0) fxdata.add(lineBuffer.substring(endPos));
      else          fxdata.add("");
    }
  }
}

// deserializes mode names string into JsonArray
// also removes WLED-SR extensions (@...) from deserialised names
void serializeModeNames(JsonArray arr) {
  for (size_t i = 0; i < strip.getModeCount(); i++) {
    String lineBuffer = FPSTR(strip.getModeData(i));
    if (lineBuffer.length() > 0) {
      uint8_t endPos = lineBuffer.indexOf('@');
      if (endPos>0) arr.add(lineBuffer.substring(0, endPos));
      else          arr.add(lineBuffer);
    }
  }
}

void serveJson(AsyncWebServerRequest* request)
{
  byte subJson = 0;
  const String& url = request->url();
  if      (url.indexOf("state") > 0) subJson = 1;
  else if (url.indexOf("info")  > 0) subJson = 2;
  else if (url.indexOf("si")    > 0) subJson = 3;
  else if (url.indexOf("nodes") > 0) subJson = 4;
  else if (url.indexOf("palx")  > 0) subJson = 5;
  else if (url.indexOf("fxda")  > 0) subJson = 6;
  #ifdef WLED_ENABLE_JSONLIVE
  else if (url.indexOf("live")  > 0) {
    serveLiveLeds(request);
    return;
  }
  #endif
  else if (url.indexOf(F("eff")) > 0) {
    // this serves just effect names without FX data extensions in names
    if (requestJSONBufferLock(19)) {
      AsyncJsonResponse* response = new AsyncJsonResponse(&doc, true);  // array document
      JsonArray lDoc = response->getRoot();
      serializeModeNames(lDoc); // remove WLED-SR extensions from effect names
      response->setLength();
      request->send(response);
      releaseJSONBufferLock();
    } else {
      //request->send_P(200, "application/json", JSON_mode_names);
      request->send(503, "application/json", F("{\"error\":3}"));
    }
    return;
  }
  else if (url.indexOf("pal") > 0) {
    request->send_P(200, "application/json", JSON_palette_names);
    return;
  }
  else if (url.indexOf("cfg") > 0 && handleFileRead(request, "/cfg.json")) {
    return;
  }
  else if (url.length() > 6) { //not just /json
    request->send(501, "application/json", F("{\"error\":\"Not implemented\"}"));
    return;
  }

  if (!requestJSONBufferLock(17)) {
    request->send(503, "application/json", F("{\"error\":3}"));
    return;
  }
  AsyncJsonResponse *response = new AsyncJsonResponse(&doc, subJson==6);

  JsonVariant lDoc = response->getRoot();

  switch (subJson)
  {
    case 1: //state
      serializeState(lDoc); break;
    case 2: //info
      serializeInfo(lDoc); break;
    case 4: //node list
      serializeNodes(lDoc); break;
    case 5: //palettes
      serializePalettes(lDoc, request); break;
    case 6: // FX helper data
      serializeModeData(lDoc.as<JsonArray>()); break;
    default: //all
      JsonObject state = lDoc.createNestedObject("state");
      serializeState(state);
      JsonObject info = lDoc.createNestedObject("info");
      serializeInfo(info);
      if (subJson != 3)
      {
        //lDoc[F("effects")]  = serialized((const __FlashStringHelper*)JSON_mode_names);
        JsonArray effects = lDoc.createNestedArray(F("effects"));
        serializeModeNames(effects); // remove WLED-SR extensions from effect names
        lDoc[F("palettes")] = serialized((const __FlashStringHelper*)JSON_palette_names);
      }
  }

  DEBUG_PRINTF("JSON buffer size: %u for request: %d\n", lDoc.memoryUsage(), subJson);

  response->setLength();
  request->send(response);
  releaseJSONBufferLock();
}

#ifdef WLED_ENABLE_JSONLIVE
#define MAX_LIVE_LEDS 180

bool serveLiveLeds(AsyncWebServerRequest* request, uint32_t wsClient)
{
  #ifdef WLED_ENABLE_WEBSOCKETS
  AsyncWebSocketClient * wsc = nullptr;
  if (!request) { //not HTTP, use Websockets
    wsc = ws.client(wsClient);
    if (!wsc || wsc->queueLength() > 0) return false; //only send if queue free
  }
  #endif

  uint16_t used = strip.getLengthTotal();
  uint16_t n = (used -1) /MAX_LIVE_LEDS +1; //only serve every n'th LED if count over MAX_LIVE_LEDS
  char buffer[2000];
  strcpy_P(buffer, PSTR("{\"leds\":["));
  obuf = buffer;
  olen = 9;

  for (size_t i= 0; i < used; i += n)
  {
    uint32_t c = strip.getPixelColor(i);
    uint8_t r = qadd8(W(c), R(c)); //add white channel to RGB channels as a simple RGBW -> RGB map
    uint8_t g = qadd8(W(c), G(c));
    uint8_t b = qadd8(W(c), B(c));
    olen += sprintf(obuf + olen, "\"%06X\",", RGBW32(r,g,b,0));
  }
  olen -= 1;
  oappend((const char*)F("],\"n\":"));
  oappendi(n);
  oappend("}");
  if (request) {
    request->send(200, "application/json", buffer);
  }
  #ifdef WLED_ENABLE_WEBSOCKETS
  else {
    wsc->text(obuf, olen);
  }
  #endif
  return true;
}
#endif