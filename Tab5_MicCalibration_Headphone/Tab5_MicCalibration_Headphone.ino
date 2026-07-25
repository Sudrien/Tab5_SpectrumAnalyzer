/*
  Tab5 Mic Calibration — Headphone / Relative
  ===========================================
  Companion to Tab5_SpectrumAnalyzer, for the frequency range the piezo
  sketch cannot reach (below ~5 kHz, and in fact the whole band).

  The piezo is a resonant emitter: loud and clean only near 4-5 kHz. A
  headphone or earbud driver (e.g. Koss Porta Pro) is a broadband dynamic
  driver with a reasonably smooth response across the whole audible range,
  so it can excite low frequencies the piezo cannot. What you lose is an
  absolute SPL reference — this sketch measures RELATIVE response only. It
  tells you the SHAPE of the mic's frequency response (how much each band
  is emphasised or suppressed relative to the others), not dB SPL.

  Pair it with the piezo sketch's single clean anchor near 5 kHz if you
  want to pin the relative curve to an absolute level.

  ---------------------------------------------------------------------
  HOW IT WORKS
  ---------------------------------------------------------------------
  There is no GPIO tone here. You play tones INTO the room from a separate
  device (phone, laptop, signal-generator app) through the headphone, held
  at a fixed position near the mic. The sketch listens, finds the dominant
  tone in each capture, and reports its frequency and level. Step a tone
  generator through a set of frequencies; the sketch logs each.

  Two modes, chosen by MODE below:

    MODE_TRACK  — continuous. Every capture, print the loudest tone's
                  frequency and level. Good for sweeping by hand and
                  watching live. Also drawn on screen.

    MODE_TABLE  — you name the frequencies in sweepPoints[]. The sketch
                  waits until it detects a strong, steady tone near each
                  target in turn, captures it, and moves on. Produces a
                  clean relative-response table at the end. Advance is
                  automatic on detection; no serial input required.

  ---------------------------------------------------------------------
  SETUP
  ---------------------------------------------------------------------
  1. A tone-generator app on a phone or laptop. Many free ones do a
     stepped sweep or single tones. A pure sine is best; the cleaner the
     tone, the better.

  2. Headphone driver held at a FIXED, repeatable distance from the mic
     port. Same rig discipline as the piezo: near-field level changes
     fast with distance, so anything that drifts between tones shows up
     as response error. One earcup facing the mic, taped or clamped a
     couple of cm away, works. Keep it there for the whole sweep.

  3. Pick a comfortable, CONSTANT volume on the source and do not touch
     it during a sweep. The absolute level is arbitrary; what must stay
     fixed is that the source outputs the same level at every frequency.
     (A real headphone is not perfectly flat, so the result includes the
     driver's own response too. For relative mic shape this is usually
     close enough; a driver with a published flat-ish curve is better.)

  4. Serial uses hardware CDC on this chip. Build with USBMode=hwcdc, the
     same flag the piezo calibration sketch needs, or you will get no
     serial output (the screen still works):

       arduino-cli compile \
         --fqbn "esp32:esp32:esp32p4:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" .
       arduino-cli upload \
         --fqbn "esp32:esp32:esp32p4:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" \
         --port /dev/ttyACM0 .

     Then:  tio -b 115200 /dev/ttyACM0

  ---------------------------------------------------------------------
  KEEP THE AUDIO CHAIN IDENTICAL
  ---------------------------------------------------------------------
  SAMPLE_RATE, FFT_SIZE and every mic_config_t field must match the
  analyzer and the piezo sketch, or the numbers will not transfer.
*/

#include <M5Unified.h>
#include <arduinoFFT.h>

// ---------- Mode ----------
#define MODE_TRACK 0
#define MODE_TABLE 1
#define MODE       MODE_TABLE

// ---------- Audio, must match the analyzer ----------
constexpr int SAMPLE_RATE = 48000;
constexpr int FFT_SIZE    = 1024;

// ---------- Measurement ----------
constexpr int   FRAMES_PER_POINT = 96;    // averaged per captured point
constexpr int   BIN_HALF_WIDTH   = 3;     // bins summed around a peak
constexpr float DETECT_SNR_DB    = 15.0f; // tone must beat the band noise by this
constexpr float TARGET_TOLERANCE = 0.06f; // detected freq within +/-6% of target
constexpr int   STEADY_HITS      = 3;     // consecutive detections before capturing

// ---------- Sweep targets (MODE_TABLE). Log-spaced across the range the
//            analyzer displays. No SPL reference — relative only. ----------
uint32_t sweepPoints[] = {
    50,   80,  125,  200,  315,  500,  800,
  1250, 2000, 3150, 5000, 8000, 12000, 16000, 20000
};
constexpr int NUM_SWEEP = sizeof(sweepPoints) / sizeof(sweepPoints[0]);

// =====================================================================

float   vReal[FFT_SIZE];
float   vImag[FFT_SIZE];
int16_t rawSamples[FFT_SIZE];

ArduinoFFT<float> FFT(vReal, vImag, FFT_SIZE, (float)SAMPLE_RATE);

constexpr int   USABLE_BINS = FFT_SIZE / 2;
constexpr float HZ_PER_BIN  = (float)SAMPLE_RATE / (float)FFT_SIZE;

float avgMag[USABLE_BINS];

// MODE_TABLE results.
float capturedLevel[NUM_SWEEP];   // measured band level, dB, or NAN
bool  captured[NUM_SWEEP];

// ---------------------------------------------------------------------

static void captureSpectrum() {
  while (!M5.Mic.record(rawSamples, FFT_SIZE, SAMPLE_RATE)) delay(1);
  for (int i = 0; i < FFT_SIZE; i++) { vReal[i] = (float)rawSamples[i]; vImag[i] = 0.0f; }
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
}

static void captureAveraged(int frames) {
  for (int i = 0; i < USABLE_BINS; i++) avgMag[i] = 0.0f;
  for (int f = 0; f < frames; f++) {
    captureSpectrum();
    for (int i = 0; i < USABLE_BINS; i++) avgMag[i] += vReal[i];
    M5.update(); delay(1); yield();
  }
  for (int i = 0; i < USABLE_BINS; i++) avgMag[i] /= (float)frames;
}

static float bandLevelDb(const float* s, float centerHz) {
  if (centerHz <= 0.0f || centerHz >= SAMPLE_RATE / 2.0f) return NAN;
  int c  = (int)roundf(centerHz / HZ_PER_BIN);
  int lo = max(1, c - BIN_HALF_WIDTH);
  int hi = min(USABLE_BINS - 1, c + BIN_HALF_WIDTH);
  float p = 0.0f;
  for (int i = lo; i <= hi; i++) p += s[i] * s[i];
  return 20.0f * log10f(sqrtf(p) + 1.0f);
}

// Find the loudest bin in the current avgMag (skipping DC) and return its
// frequency and level. Uses a quick parabolic interpolation around the
// peak so the reported frequency is not quantised to the bin grid.
static void dominantTone(float& freqOut, float& levelOut) {
  int peak = 1;
  for (int i = 2; i < USABLE_BINS; i++) if (avgMag[i] > avgMag[peak]) peak = i;

  float f = (float)peak;
  if (peak > 1 && peak < USABLE_BINS - 1) {
    float a = avgMag[peak - 1], b = avgMag[peak], c = avgMag[peak + 1];
    float denom = (a - 2.0f * b + c);
    if (fabsf(denom) > 1e-6f) f += 0.5f * (a - c) / denom;
  }
  freqOut  = f * HZ_PER_BIN;
  levelOut = bandLevelDb(avgMag, peak * HZ_PER_BIN);
}

// Rough broadband noise estimate for a quick SNR gate: median-ish via the
// mean of the lower half of bin magnitudes. Cheap and good enough to tell
// "a tone is present" from "silence".
static float roughNoiseDb() {
  float sum = 0.0f; int n = 0;
  for (int i = 1; i < USABLE_BINS; i++) { sum += avgMag[i]; n++; }
  float mean = sum / n;
  return 20.0f * log10f(mean + 1.0f);
}

static void showStatus(const char* l1, const char* l2) {
  M5.Display.fillRect(0, 100, M5.Display.width(), 140, TFT_BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setCursor(20, 110); M5.Display.print(l1);
  M5.Display.setCursor(20, 160); M5.Display.print(l2);
}

// ---------------------------------------------------------------------

#if MODE == MODE_TABLE

static void runSweep() {
  char l1[64], l2[64];

  Serial.println();
  Serial.println("# Tab5 headphone relative-response sweep");
  Serial.printf("# sample_rate=%d fft_size=%d bin_hz=%.2f\n",
                SAMPLE_RATE, FFT_SIZE, HZ_PER_BIN);
  Serial.println("# RELATIVE levels only (arbitrary reference), NOT dB SPL.");
  Serial.println("# Play a steady tone near each target; capture is automatic.");
  Serial.println();
  Serial.println("SWEEP,target_hz,detected_hz,level_db");

  for (int i = 0; i < NUM_SWEEP; i++) { capturedLevel[i] = NAN; captured[i] = false; }

  for (int i = 0; i < NUM_SWEEP; i++) {
    uint32_t target = sweepPoints[i];
    snprintf(l1, sizeof(l1), "Play %lu Hz", (unsigned long)target);
    snprintf(l2, sizeof(l2), "point %d/%d", i + 1, NUM_SWEEP);
    showStatus(l1, l2);

    // Wait for a strong, steady tone near this target.
    int hits = 0;
    while (hits < STEADY_HITS) {
      M5.update();
      captureAveraged(FRAMES_PER_POINT / 3);   // quick look while waiting

      float f, lvl; dominantTone(f, lvl);
      float snr = lvl - roughNoiseDb();
      bool near = fabsf(f - target) <= target * TARGET_TOLERANCE;

      if (near && snr >= DETECT_SNR_DB) hits++;
      else hits = 0;

      // Let a tap skip a frequency you cannot produce.
      auto t = M5.Touch.getDetail();
      if (t.wasPressed()) {
        Serial.printf("SWEEP,%lu,skipped,skipped\n", (unsigned long)target);
        snprintf(l2, sizeof(l2), "skipped");
        showStatus(l1, l2);
        delay(400);
        goto next_point;
      }

      snprintf(l2, sizeof(l2), "hear %d Hz  snr %d", (int)f, (int)snr);
      showStatus(l1, l2);
    }

    // Steady tone confirmed — take a clean averaged capture.
    {
      captureAveraged(FRAMES_PER_POINT);
      float f, lvl; dominantTone(f, lvl);
      capturedLevel[i] = lvl;
      captured[i]      = true;
      Serial.printf("SWEEP,%lu,%d,%.1f\n", (unsigned long)target, (int)f, lvl);
      snprintf(l2, sizeof(l2), "got %.1f dB", lvl);
      showStatus(l1, l2);
      delay(300);
    }
    next_point:;
  }

  // --- Relative curve: normalise so the max is 0 dB ---
  Serial.println();
  Serial.println("# --- relative response ---");
  float maxLvl = -1e9f;
  for (int i = 0; i < NUM_SWEEP; i++)
    if (captured[i] && capturedLevel[i] > maxLvl) maxLvl = capturedLevel[i];

  if (maxLvl < -1e8f) {
    Serial.println("# nothing captured.");
    showStatus("Sweep done", "no points");
    return;
  }

  Serial.println("# freq_hz  rel_db   (0 = loudest band; negative = quieter)");
  for (int i = 0; i < NUM_SWEEP; i++) {
    if (!captured[i]) {
      Serial.printf("# %7lu   (skipped)\n", (unsigned long)sweepPoints[i]);
      continue;
    }
    Serial.printf("# %7lu  %6.1f\n",
                  (unsigned long)sweepPoints[i], capturedLevel[i] - maxLvl);
  }

  // Paste-ready: relative correction is the NEGATIVE of the response, so
  // applying it flattens the measured curve.
  Serial.println();
  Serial.println("# Relative correction (negate response). Anchor to absolute");
  Serial.println("# SPL separately using the piezo's ~5 kHz point if wanted.");
  Serial.println("struct CalPoint { float hz; float offsetDb; };");
  Serial.println("static const CalPoint CAL_TABLE_REL[] = {");
  for (int i = 0; i < NUM_SWEEP; i++) {
    if (!captured[i]) continue;
    Serial.printf("  { %8.1ff, %7.2ff },\n",
                  (float)sweepPoints[i], -(capturedLevel[i] - maxLvl));
  }
  Serial.println("};");
  Serial.println();

  showStatus("Sweep complete", "tap to repeat");
}

#endif  // MODE_TABLE

// ---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5.begin(cfg);

  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = SAMPLE_RATE;
  mic_cfg.stereo      = false;
  M5.Mic.config(mic_cfg);
  M5.Mic.begin();

  M5.Display.setRotation(3);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_GREEN);
  M5.Display.setTextSize(4);
  M5.Display.drawString("Mic Cal (headphone)", 20, 20);

  delay(1500);

#if MODE == MODE_TABLE
  runSweep();
#else
  Serial.println("# Tab5 headphone tracker — loudest tone each capture");
  Serial.println("# RELATIVE level, arbitrary reference.");
  Serial.println("TRACK,detected_hz,level_db");
#endif
}

void loop() {
  M5.update();

#if MODE == MODE_TABLE
  auto t = M5.Touch.getDetail();
  if (t.wasPressed()) runSweep();
  delay(20);
#else
  // Continuous tracker.
  captureAveraged(FRAMES_PER_POINT / 2);
  float f, lvl; dominantTone(f, lvl);
  float snr = lvl - roughNoiseDb();

  M5.Display.fillRect(20, 120, 700, 120, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE);
  if (snr >= DETECT_SNR_DB) {
    Serial.printf("TRACK,%d,%.1f\n", (int)f, lvl);
    M5.Display.setTextSize(6);
    M5.Display.setCursor(20, 120);
    M5.Display.printf("%d Hz", (int)f);
    M5.Display.setTextSize(3);
    M5.Display.setCursor(20, 200);
    M5.Display.printf("%.1f dB", lvl);
  } else {
    M5.Display.setTextSize(3);
    M5.Display.setCursor(20, 150);
    M5.Display.print("(listening...)");
  }
  delay(50);
#endif
}

/*
  ---------------------------------------------------------------------
  USING THE RESULTS
  ---------------------------------------------------------------------
  MODE_TABLE prints CAL_TABLE_REL[], a relative correction curve: negate
  of the measured response, normalised so the loudest band is 0. Applying
  it flattens the mic's frequency-response shape. It carries NO absolute
  level — every value is relative to the loudest band in the sweep.

  To combine with the piezo sketch's absolute anchor:
    1. Take the piezo's single clean offset near 5 kHz — call it A_5k.
    2. This sweep gives a relative correction at 5 kHz — call it R_5k.
    3. Shift the whole relative curve by (A_5k - R_5k). Now the relative
       shape is pinned to the piezo's absolute level, and the combined
       table is both correctly shaped and correctly positioned.

  Then interpolate the combined table by frequency in the analyzer, the
  same way described in the piezo sketch's notes.

  Caveats. The headphone's own response is baked into this measurement;
  a driver with a flatter published curve gives a cleaner result. And the
  detected-frequency gate assumes ONE dominant tone at a time — play
  single tones, not chords or music.
*/
