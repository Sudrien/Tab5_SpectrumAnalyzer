/*
  Tab5 Mic Calibration  (piezo, ~5-10 kHz)
  ========================================
  Companion to Tab5_SpectrumAnalyzer. Drives a piezo element from a GPIO
  at a series of known frequencies, measures what the mic + FFT chain
  reports at each, and prints the results as CSV over serial. The final
  block is a paste-ready C table of correction offsets.

  Scope: this covers only ~5-10 kHz, the piezo's clean region. Below that
  the element is too quiet and its harmonics swamp the fundamental, so it
  cannot measure the mic there. For the low end use the headphone-based
  sketch (Tab5_MicCalibration_Headphone) instead.

  Each frequency is measured over several full runs (RUNS_TO_AVERAGE); the
  reported offset is the mean, and the run-to-run SPREAD is printed per
  frequency. Spread is the honest quality metric — a point that wanders
  more than a dB or two between runs means the acoustic rig is moving, and
  no within-run averaging fixes that. Only points stable across runs make
  it into the paste-ready table.

  This sketch measures. It does not modify the analyzer. Integration is
  manual and deliberate — see "USING THE RESULTS" at the bottom.

  ---------------------------------------------------------------------
  HARDWARE
  ---------------------------------------------------------------------
  Tested against a TDK PS1240P02BT piezo element (or equivalent):
      resonant frequency  4 kHz
      rated output        70 dBA @ 10 cm, at 3 Vo-p square wave
      max input           30 Vo-p
      test conditions     anechoic chamber, A-weighted, 25C / 60% RH

  3.3 V logic sits just above the datasheet's 3 Vo-p test condition, so a
  GPIO drives this part directly with no transistor. Wire one leg to the
  GPIO, the other to GND. A ~100 ohm series resistor is optional; it
  limits edge current into what is essentially a small capacitor, at the
  cost of a fraction of a dB.

  Do NOT use a buzzer with a built-in oscillator. This needs the bare
  externally-driven element, or the frequency is whatever the internal
  oscillator decides.

  ---------------------------------------------------------------------
  SETUP
  ---------------------------------------------------------------------
  1. Wire the piezo to ExtPort1 (bottom edge of the board). One leg to
     G1, the other to the GND pin beside it. PIEZO_PIN is already set to
     G1. If you need a different pin, ExtPort1 also exposes G0, G49 and
     G50 — avoid G0 (strapping pin) and stay off Port A's G53/G54, which
     are the shared I2C bus. A ~100 ohm series resistor is optional; it
     limits edge current into what is essentially a small capacitor, at
     the cost of a fraction of a dB.

  2. Find the mic ports on the case. The Tab5 has a dual-mic array; the
     one M5Unified reads is a single channel of it. If you are unsure
     which opening is live, run the analyzer and scratch near each.

  3. Fix the piezo 10 cm from the mic port, on-axis, and make it rigid.
     Tape, a clamp, a bit of foamcore with two holes — anything that does
     not move between test tones. Near-field SPL changes fast with
     distance; a centimetre of drift is worth more error than everything
     else in this procedure combined. If you can only hold it by hand,
     the relative curve is still useful but the absolute numbers are not.

  4. Quiet room, soft furnishings if possible. The datasheet figure comes
     from an anechoic chamber. A normal room adds reflections and standing
     waves that vary with frequency, and a few dB of disagreement at any
     given point is expected, not a bug.

  5. Build with USBMode=hwcdc. On this ESP32-P4 the default cdc routing
     gives the screen but NO serial to the host; hwcdc routes Serial to
     the USB-Serial/JTAG peripheral that reaches the host (the screen goes
     blank in that mode, which is fine here — this sketch only needs
     serial). The analyzer stays on plain cdc because it needs the screen.

         arduino-cli compile --fqbn "esp32:esp32:esp32p4:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" .
         arduino-cli upload  --fqbn "esp32:esp32:esp32p4:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" --port /dev/ttyACM0 .

     Then watch the serial output:
         tio -b 115200 /dev/ttyACM0
     Or capture just the data to a file:
         tio -b 115200 --log --log-file cal.log /dev/ttyACM0
     then afterward:  grep '^CAL' cal.log

     Each trigger now runs RUNS_TO_AVERAGE full sweeps and reports the mean
     offset plus run-to-run spread per frequency. It runs once at boot and
     re-runs on a timer; tapping the screen forces an immediate run. Serial
     'r' also triggers one if input happens to work — do not count on it.

  6. Watch the SPREAD column in the summary. A point stable to within a dB
     or two across runs is trustworthy; one that swings more than that
     means the rig is moving and that point is dropped from the table.

  ---------------------------------------------------------------------
  IMPORTANT: KEEP THE AUDIO CHAIN IDENTICAL
  ---------------------------------------------------------------------
  SAMPLE_RATE, FFT_SIZE, the window function, and every field of
  mic_config_t must match the analyzer exactly, or the offsets measured
  here will not apply there. In particular M5Unified's `magnification`
  (default 16) is a plain digital multiply and `noise_filter_level`
  (default 0, off) is a one-pole IIR — both silently rescale everything
  if changed. Neither is touched here, and neither should be touched
  there.

  ---------------------------------------------------------------------
  A CAVEAT ABOUT SQUARE WAVES
  ---------------------------------------------------------------------
  A square wave at f also contains 3f, 5f, 7f... and this element has a
  sharp resonance near 4-5 kHz. Driving at 1 kHz therefore radiates a
  substantial 5th harmonic sitting right on resonance, which can exceed
  the fundamental. The datasheet's square-wave curve was measured through
  an A-weighting filter into a recorder, i.e. broadband, so it includes
  that harmonic content.

  This sketch reports the fundamental, the 3rd and 5th harmonics, and the
  broadband total separately for exactly that reason. Where FUND and
  BROADBAND agree, the fundamental dominates and the datasheet comparison
  is meaningful. Where BROADBAND is well above FUND, the element is
  mostly emitting harmonics and the datasheet value for that frequency
  should not be treated as a calibration anchor for that bin.

  The cleanest anchor is near resonance (4-5 kHz), where the fundamental
  dominates and output is loudest relative to room noise.
*/

#include <M5Unified.h>
#include <arduinoFFT.h>

// =====================================================================
// CONFIG
// =====================================================================

// Pin driving the piezo. Verified against the C145 Tab5 pinmap.
//
// ExtPort1 (bottom edge of the board) breaks out four plain ESP32-P4
// GPIOs — G0, G1, G49, G50 — alongside 3V3, EXT5V and GND. G1 is used
// here: it sits right next to the 3V3 and GND pins on that header, so
// the piezo wires land on adjacent contacts, and unlike G0 it carries
// no strapping/boot role.
//
// Do NOT move this to Port A (G53/G54). Those are the M5 Grove Port A
// I2C pins, shared with the IO expanders, RTC, IMU and touch controller.
// A GPIO-toggled load has no business on that bus.
#define PIEZO_PIN 1

// Must match Tab5_SpectrumAnalyzer exactly.
constexpr int SAMPLE_RATE = 48000;
constexpr int FFT_SIZE    = 1024;

// Measurement behaviour.
constexpr int   FRAMES_PER_POINT = 96;    // averaged per frequency (was 24;
                                          // more frames pulls each reading
                                          // further out of the noise)
constexpr int   DISCARD_FRAMES   = 4;     // dropped after each tone change
constexpr int   SETTLE_MS        = 250;   // piezo + room settling time
constexpr int   BIN_HALF_WIDTH   = 3;     // bins summed either side of a peak
constexpr float MIN_USABLE_SNR   = 12.0f; // dB over noise floor to trust a point

// Multi-run stability. Each frequency is measured RUNS_TO_AVERAGE times;
// the printed offset is the mean, and the spread (max-min) across runs is
// reported per frequency. Run-to-run spread is the real quality metric
// here — if a point bounces by more than a dB or two, the acoustic rig is
// moving and no amount of averaging within a run will save it.
constexpr int RUNS_TO_AVERAGE = 5;

// Behaviour after the first automatic run. Because USB-CDC serial input
// is unreliable on this chip (keystrokes may never reach Serial.read),
// the sketch does not depend on it: it re-runs on a timer, and a screen
// tap triggers a run immediately. Serial 'r' still works if input does.
constexpr bool     AUTO_REPEAT      = true;
constexpr uint32_t AUTO_REPEAT_MS   = 8000;   // gap between automatic runs

// Test frequencies and the datasheet's square-wave SPL at 10 cm.
//
// Trimmed to the piezo's clean region. Below ~5 kHz this element is too
// quiet and its square-wave harmonics land on its own 4-5 kHz resonance
// and swamp the fundamental, so those points measured the piezo's
// resonance rather than the mic and are not recoverable. The bass end
// needs a different source — see Tab5_MicCalibration_Headphone.
//
// These SPL values were read off the printed frequency-response chart by
// eye and are worth no better than +/-2-3 dB. Re-read the chart yourself
// and correct them — every offset this sketch computes inherits their
// error directly. Set an entry to NAN to measure a frequency without
// claiming a reference value for it.
struct TestPoint {
  uint32_t hz;
  float    datasheetSpl;   // dB @ 10cm, square wave drive, or NAN
};

TestPoint testPoints[] = {
  {  5000,  77.0f },   // main resonance, loudest and cleanest
  {  5500,  74.0f },
  {  6000,  65.0f },
  {  6500,  67.0f },
  {  7000,  70.0f },
  {  8000,  65.0f },
  {  9000,  62.0f },
  { 10000,  60.0f },
};
constexpr int NUM_POINTS = sizeof(testPoints) / sizeof(testPoints[0]);

// =====================================================================

float   vReal[FFT_SIZE];
float   vImag[FFT_SIZE];
int16_t rawSamples[FFT_SIZE];

ArduinoFFT<float> FFT(vReal, vImag, FFT_SIZE, (float)SAMPLE_RATE);

constexpr int   USABLE_BINS = FFT_SIZE / 2;
constexpr float HZ_PER_BIN  = (float)SAMPLE_RATE / (float)FFT_SIZE;

// Accumulated magnitude spectrum, averaged over FRAMES_PER_POINT frames.
float avgMag[USABLE_BINS];

// Averaged magnitude spectrum captured once with the tone OFF, so each
// tone's band can be compared against the noise IN THAT SAME BAND. The
// earlier code compared a 7-bin tone against a 512-bin broadband sum,
// which is not a like-for-like comparison and produced a meaningless
// ~77 dB "floor" in a silent room.
float noiseMag[USABLE_BINS];

// Results, kept so the summary table can be printed at the end.
float measuredFund[NUM_POINTS];
float measuredBroad[NUM_POINTS];
float measuredSnr[NUM_POINTS];
float offsetDb[NUM_POINTS];
bool  pointUsable[NUM_POINTS];

// Multi-run accumulators, one slot per frequency.
float offAccum[NUM_POINTS];   // sum of offsets across runs, for the mean
float offMin[NUM_POINTS];     // smallest offset seen across runs
float offMax[NUM_POINTS];     // largest offset seen across runs
int   offCount[NUM_POINTS];   // runs that produced a usable reading here

// ---------------------------------------------------------------------

static void toneOn(uint32_t hz) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIEZO_PIN, hz);
#else
  ledcWriteTone(0, hz);
#endif
}

static void toneOff() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIEZO_PIN, 0);
  ledcWrite(PIEZO_PIN, 0);
#else
  ledcWriteTone(0, 0);
  ledcWrite(0, 0);
#endif
}

// One capture + FFT, magnitudes left in vReal[0..USABLE_BINS).
static void captureSpectrum() {
  while (!M5.Mic.record(rawSamples, FFT_SIZE, SAMPLE_RATE)) delay(1);
  for (int i = 0; i < FFT_SIZE; i++) {
    vReal[i] = (float)rawSamples[i];
    vImag[i] = 0.0f;
  }
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
}

// Average several spectra into avgMag to pull the noise down.
static void captureAveraged(int frames) {
  for (int i = 0; i < USABLE_BINS; i++) avgMag[i] = 0.0f;
  for (int f = 0; f < frames; f++) {
    captureSpectrum();
    for (int i = 0; i < USABLE_BINS; i++) avgMag[i] += vReal[i];
    // Yield between frames. Long back-to-back capture loops otherwise
    // starve the task watchdog and trigger HP_SYS_HP_WDT_RESET mid-run,
    // which on native USB-CDC drops the serial port and loses output.
    M5.update();
    delay(1);
    yield();
  }
  for (int i = 0; i < USABLE_BINS; i++) avgMag[i] /= (float)frames;
}

// Band level around centerHz, read out of an arbitrary spectrum array.
// Summing power over the few bins a windowed tone smears across beats
// reading a single bin. Returns the analyzer's dB scale: 20*log10(x+1).
static float bandLevelDb(const float* spectrum, float centerHz) {
  if (centerHz <= 0.0f || centerHz >= SAMPLE_RATE / 2.0f) return NAN;
  int center = (int)roundf(centerHz / HZ_PER_BIN);
  int lo = max(1, center - BIN_HALF_WIDTH);
  int hi = min(USABLE_BINS - 1, center + BIN_HALF_WIDTH);
  float power = 0.0f;
  for (int i = lo; i <= hi; i++) power += spectrum[i] * spectrum[i];
  return 20.0f * log10f(sqrtf(power) + 1.0f);
}

// Convenience: band level in the current avgMag spectrum.
static float levelDbAround(float centerHz) {
  return bandLevelDb(avgMag, centerHz);
}

// Total level across the whole spectrum, DC excluded.
static float broadbandDb() {
  float power = 0.0f;
  for (int i = 1; i < USABLE_BINS; i++) power += avgMag[i] * avgMag[i];
  return 20.0f * log10f(sqrtf(power) + 1.0f);
}

static void showStatus(const char* line1, const char* line2) {
  M5.Display.fillRect(0, 100, M5.Display.width(), 120, TFT_BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setCursor(20, 110);
  M5.Display.print(line1);
  M5.Display.setCursor(20, 160);
  M5.Display.print(line2);
}

// ---------------------------------------------------------------------

// One full physical pass: measure the noise floor, then sweep every test
// frequency once. Fills measuredFund/Broad/Snr/offsetDb/pointUsable.
// runIndex/runTotal are for the on-screen and serial progress labels.
static void measurePass(int runIndex, int runTotal) {
  char l1[64], l2[64];

  // --- Noise floor, tone off. Store the whole spectrum so each tone's
  //     band can be compared against the noise in the same band. ---
  snprintf(l1, sizeof(l1), "Noise floor");
  snprintf(l2, sizeof(l2), "run %d/%d, keep quiet", runIndex, runTotal);
  showStatus(l1, l2);
  toneOff();
  delay(SETTLE_MS * 2);
  captureAveraged(FRAMES_PER_POINT);
  for (int i = 0; i < USABLE_BINS; i++) noiseMag[i] = avgMag[i];

  for (int p = 0; p < NUM_POINTS; p++) {
    uint32_t hz = testPoints[p].hz;

    snprintf(l1, sizeof(l1), "Tone %lu Hz", (unsigned long)hz);
    snprintf(l2, sizeof(l2), "run %d/%d, point %d/%d",
             runIndex, runTotal, p + 1, NUM_POINTS);
    showStatus(l1, l2);

    toneOn(hz);
    delay(SETTLE_MS);
    for (int d = 0; d < DISCARD_FRAMES; d++) captureSpectrum();

    captureAveraged(FRAMES_PER_POINT);

    float fund      = levelDbAround((float)hz);
    float h3        = levelDbAround((float)hz * 3.0f);
    float h5        = levelDbAround((float)hz * 5.0f);
    float broad     = broadbandDb();
    float noiseBand = bandLevelDb(noiseMag, (float)hz);
    float snr       = fund - noiseBand;

    float ds     = testPoints[p].datasheetSpl;
    bool  hasRef = !isnan(ds);
    float off    = hasRef ? (ds - fund) : NAN;
    bool  usable = hasRef && (snr >= MIN_USABLE_SNR);

    measuredFund[p]  = fund;
    measuredBroad[p] = broad;
    measuredSnr[p]   = snr;
    offsetDb[p]      = off;
    pointUsable[p]   = usable;

    // Per-run CSV line, tagged with the run number.
    Serial.printf("CAL,%d,%lu,%.1f,", runIndex, (unsigned long)hz, fund);
    if (isnan(h3)) Serial.print("nyquist,"); else Serial.printf("%.1f,", h3);
    if (isnan(h5)) Serial.print("nyquist,"); else Serial.printf("%.1f,", h5);
    Serial.printf("%.1f,%.1f,", broad, snr);
    if (hasRef) Serial.printf("%.1f,%.1f,", ds, off);
    else        Serial.print("na,na,");
    Serial.println(usable ? "yes" : "NO");
  }

  toneOff();
}

static void runCalibration() {
  Serial.println();
  Serial.println("# Tab5 mic calibration run");
  Serial.printf("# sample_rate=%d fft_size=%d bin_hz=%.2f frames=%d runs=%d\n",
                SAMPLE_RATE, FFT_SIZE, HZ_PER_BIN, FRAMES_PER_POINT, RUNS_TO_AVERAGE);
  Serial.println("# levels are 20*log10(mag+1) on the analyzer's internal scale,");
  Serial.println("# NOT dB SPL. datasheet_spl is dB @10cm from the part's chart.");
  Serial.println("# SNR is each tone's band vs the noise in that same band.");
  Serial.println();
  Serial.println("CAL,run,freq_hz,fund_db,h3_db,h5_db,broadband_db,snr_db,datasheet_spl,offset_db,usable");

  // Reset accumulators.
  for (int p = 0; p < NUM_POINTS; p++) {
    offAccum[p] = 0.0f;
    offMin[p]   =  1e9f;
    offMax[p]   = -1e9f;
    offCount[p] = 0;
  }

  // Repeat the whole physical measurement several times.
  for (int run = 1; run <= RUNS_TO_AVERAGE; run++) {
    measurePass(run, RUNS_TO_AVERAGE);
    for (int p = 0; p < NUM_POINTS; p++) {
      if (!pointUsable[p]) continue;
      offAccum[p] += offsetDb[p];
      offMin[p]    = min(offMin[p], offsetDb[p]);
      offMax[p]    = max(offMax[p], offsetDb[p]);
      offCount[p]++;
    }
    delay(300);   // brief gap between runs
  }

  showStatus("Runs complete", AUTO_REPEAT ? "repeats; tap to force" : "tap to run again");

  // --- Summary ---
  Serial.println();
  Serial.println("# --- summary ---");
  Serial.println("# per-frequency mean offset and run-to-run spread.");
  Serial.println("# SPREAD is the real quality signal: a point that moves more");
  Serial.println("# than ~2 dB across runs means the acoustic rig is moving.");
  Serial.println();
  Serial.println("# freq_hz  runs  mean_off  spread   flag");

  int stableCount = 0;
  for (int p = 0; p < NUM_POINTS; p++) {
    if (offCount[p] == 0) {
      Serial.printf("# %7lu     0        --      --   no usable reading\n",
                    (unsigned long)testPoints[p].hz);
      continue;
    }
    float mean   = offAccum[p] / offCount[p];
    float spread = offMax[p] - offMin[p];
    const char* flag = "ok";
    if (spread > 3.0f)      flag = "UNSTABLE — rig moving?";
    else if (spread > 1.5f) flag = "marginal";
    else                    stableCount++;
    Serial.printf("# %7lu  %4d  %8.2f  %6.2f   %s\n",
                  (unsigned long)testPoints[p].hz, offCount[p], mean, spread, flag);
  }

  // Harmonic-contamination note (uses the last run's broadband/fund).
  for (int p = 0; p < NUM_POINTS; p++) {
    float excess = measuredBroad[p] - measuredFund[p];
    if (excess > 6.0f) {
      Serial.printf("# %5lu Hz: broadband %.1f dB over fundamental — harmonic-heavy.\n",
                    (unsigned long)testPoints[p].hz, excess);
    }
  }

  // --- Paste-ready table, mean offsets, stable points only ---
  int pasteCount = 0;
  for (int p = 0; p < NUM_POINTS; p++)
    if (offCount[p] > 0 && (offMax[p] - offMin[p]) <= 3.0f) pasteCount++;

  Serial.println();
  if (pasteCount == 0) {
    Serial.println("# No sufficiently stable points. Check the rig and rerun.");
    Serial.println();
    return;
  }

  Serial.printf("# %d stable point(s). Mean offsets, paste into the analyzer:\n",
                pasteCount);
  Serial.println();
  Serial.println("struct CalPoint { float hz; float offsetDb; };");
  Serial.println("static const CalPoint CAL_TABLE[] = {");
  for (int p = 0; p < NUM_POINTS; p++) {
    if (offCount[p] == 0 || (offMax[p] - offMin[p]) > 3.0f) continue;
    Serial.printf("  { %8.1ff, %7.2ff },\n",
                  (float)testPoints[p].hz, offAccum[p] / offCount[p]);
  }
  Serial.println("};");
  Serial.println();

  // Spread of the mean offsets across frequency: is one constant enough?
  float mn = 1e9f, mx = -1e9f, sum = 0.0f; int n = 0;
  for (int p = 0; p < NUM_POINTS; p++) {
    if (offCount[p] == 0 || (offMax[p] - offMin[p]) > 3.0f) continue;
    float mean = offAccum[p] / offCount[p];
    mn = min(mn, mean); mx = max(mx, mean); sum += mean; n++;
  }
  float grand = sum / n;
  Serial.printf("# across frequency: mean %.2f dB, range %.2f dB (%.2f to %.2f)\n",
                grand, mx - mn, mn, mx);
  if ((mx - mn) < 4.0f) {
    Serial.println("# Range is small — one constant offset is enough for this band:");
    Serial.printf("#   constexpr float CAL_OFFSET_DB = %.2f;\n", grand);
  } else {
    Serial.println("# Range is wide — interpolate the table across frequency.");
  }
  Serial.println();
}

// ---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5.begin(cfg);

  // Identical to the analyzer. Do not add gain or filtering here.
  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = SAMPLE_RATE;
  mic_cfg.stereo      = false;
  M5.Mic.config(mic_cfg);
  M5.Mic.begin();

  M5.Display.setRotation(3);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextSize(4);
  M5.Display.drawString("Mic Calibration", 20, 20);

  delay(1500);   // let the serial console attach before anything prints

  if (PIEZO_PIN < 0) {
    showStatus("PIEZO_PIN not set", "edit the sketch");
    Serial.println();
    Serial.println("PIEZO_PIN is unset. Set it to an ExtPort1 GPIO (G1, G49");
    Serial.println("and G50 are all fine; avoid G0) at the top of this sketch");
    Serial.println("and reflash. Stay off Port A's G53/G54 — that is the shared");
    Serial.println("I2C bus, not general-purpose GPIO.");
    return;
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIEZO_PIN, 4000, 10);
#else
  ledcSetup(0, 4000, 10);
  ledcAttachPin(PIEZO_PIN, 0);
#endif
  toneOff();

  runCalibration();
}

void loop() {
  M5.update();

  static uint32_t lastRunMs = millis();
  bool trigger = false;

  // 1) Serial 'r' — works only if USB-CDC input is actually delivering.
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'r' || c == 'R') trigger = true;
  }

  // 2) Screen tap — always available, no serial dependency.
  auto t = M5.Touch.getDetail();
  if (t.wasPressed()) trigger = true;

  // 3) Timer — runs on its own so no input path is required at all.
  if (AUTO_REPEAT && (millis() - lastRunMs >= AUTO_REPEAT_MS)) trigger = true;

  if (trigger && PIEZO_PIN >= 0) {
    runCalibration();
    lastRunMs = millis();
  }

  delay(20);
}

/*
  ---------------------------------------------------------------------
  USING THE RESULTS
  ---------------------------------------------------------------------
  The printed CAL_TABLE maps frequency to the dB that must be ADDED to a
  measured level to land on the datasheet's SPL figure.

  If the offsets are all within a few dB of each other, the mic is close
  enough to flat over this range that one constant will do. Add it to the
  analyzer's dB calculation:

      float db = 20.0f * log10f(mag + 1.0f) + CAL_OFFSET_DB;

  If the offsets vary a lot, the mic response is not flat and a single
  constant will be wrong nearly everywhere. Interpolate the table by
  frequency instead. Since the analyzer already precomputes each bar's
  band in setup(), the cheap approach is to resolve the correction once
  per bar there and store it, rather than per frame:

      float barOffset[NUM_BARS];   // filled in setup() by interpolating
                                   // CAL_TABLE at each bar's centre freq

  ...then apply barOffset[b] where the analyzer computes db.

  Either way, MIN_DB and MAX_DB then become real numbers rather than
  arbitrary ones, and the axis can honestly be labelled dB SPL.

  Two limits worth remembering. This only calibrates the range the piezo
  actually covers, roughly 1-10 kHz; outside it the axis remains
  arbitrary, and filling in the rest needs a different source such as a
  headphone driver with a known response. And the whole chain is anchored
  to a datasheet minimum figure measured in an anechoic chamber, so treat
  the absolute numbers as approximate. The shape of the curve is far more
  trustworthy than its absolute height.
*/
