#include <Arduino.h>
#include <ESP32Servo.h>
#include <HomeSpan.h>
#include <TFT_eSPI.h>

// T-Display-S3 holds its own power rail on when running from battery.
constexpr uint8_t PIN_PWR_ON = 15;

constexpr uint8_t PIN_SERVO_A = 1;
constexpr uint8_t PIN_SERVO_B = 2;

// Both onboard buttons toggle the faceplate; they pull to GND when pressed.
constexpr uint8_t PIN_BTN_BOOT = 0;
constexpr uint8_t PIN_BTN_14 = 14;
constexpr uint32_t DEBOUNCE_MS = 40;

// MG90S: 50 Hz frame. 500-2400us maps to 0-180 deg; the absolute limits leave a
// little headroom past that so the closing press below has somewhere to go.
constexpr int SERVO_FREQ_HZ = 50;
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2400;
// Derive both limits from one headroom value. They MUST stay symmetric: the two
// servos press in opposite directions, so an uneven clamp makes one side travel
// further than the other and racks the faceplate until it jams.
// 140us is about 13 degrees, which is the ceiling for CLOSE_PRESS_DEG.
constexpr int SERVO_US_HEADROOM = 160;
constexpr int SERVO_ABS_MIN_US = SERVO_MIN_US - SERVO_US_HEADROOM;
constexpr int SERVO_ABS_MAX_US = SERVO_MAX_US + SERVO_US_HEADROOM;

// Faceplate travel. The two servos sit on opposite sides, so their ranges
// run opposite ways. Swap CLOSED/OPEN per servo if a horn is mounted mirrored.
constexpr float SERVO_A_CLOSED_DEG = 180.0f;
constexpr float SERVO_A_OPEN_DEG = 90.0f;
constexpr float SERVO_B_CLOSED_DEG = 0.0f;
constexpr float SERVO_B_OPEN_DEG = 90.0f;

// Time for a full closed <-> open move; partial moves are scaled down.
// The MG90S needs roughly 150-200ms to cover 90 degrees, so values below
// ~250ms just get clipped by the servo's own top speed.
// Opening and closing are timed separately. Closing is deliberately quick so the
// plate carries momentum into the stop and seats itself; a slow close just leans
// against the stop and leaves a gap. The MG90S tops out near 600 deg/s, and the
// full sweep is about 104 deg, so roughly 180ms is the floor for the close.
constexpr uint32_t OPEN_TIME_MS = 250;
constexpr uint32_t CLOSE_TIME_MS = 250;

// Once closed the faceplate rests on its own stop, so holding torque is wasted
// current and causes the servos to buzz. Cut the PWM shortly after it settles.
// Never done when open: there the servos are holding the plate against gravity.
constexpr bool RELAX_WHEN_CLOSED = true;
// Time spent still pushing at the shut position before the drive is cut. Too
// short and the plate is released before it is fully driven home; too long and
// both servos sit stalled against the stop drawing ~700mA each.
constexpr uint32_t RELAX_DELAY_MS = 350;
constexpr uint32_t UPDATE_INTERVAL_MS = 20;

// The plate is held shut by its own weight resting on the stop, so the servos
// do not need to clamp it. Backing off would only open a visible gap: keep 0.
constexpr float CLOSE_BACKOFF_DEG = 0.0f;
constexpr uint32_t SEAT_TIME_MS = 150;
constexpr uint32_t BACKOFF_TIME_MS = 150;

// Closing suffers the same weak-finish problem as opening: the last steps of the
// ramp carry almost no torque. Aim a few degrees PAST the shut position so the
// servo arrives with real error left, and drives the plate home instead of
// drifting onto it. This is deliberate over-travel, so keep it small.
// Measured: the mechanism hard-stops near 13 deg past nominal closed. Aiming
// past it means the servo arrives still stalling, which is what actually drives
// the plate home; stopping short of it leaves a gap that gravity cannot close,
// because the detached gearbox has enough friction to hold the plate there.
// This pose MUST be released promptly - see RELAX_DELAY_MS. Holding it forever
// would be a permanent stall, which is why keeping the servos powered when shut
// is not an option on this mechanism.
constexpr float CLOSE_PRESS_DEG = 14.0f;

// If the press needs more pulse width than the headroom allows, one servo gets
// clamped and the other does not, and the plate jams. Fail the build instead.
static_assert(CLOSE_PRESS_DEG / 180.0f * (SERVO_MAX_US - SERVO_MIN_US) <= SERVO_US_HEADROOM,
              "CLOSE_PRESS_DEG exceeds SERVO_US_HEADROOM: raise the headroom or lower the press");

// Breaking away from the closed pose is the hard part: interpolating smoothly
// asks for ~0.15 deg per step, and that tiny position error produces almost no
// torque, so the plate can sit there buzzing instead of lifting. Commanding a
// single larger step first gives the servo a real error to work against.
// Raise this if it still struggles to start; 0 disables the kick.
constexpr float OPEN_KICK_DEG = 6.0f;
constexpr uint32_t OPEN_KICK_MS = 120;

Servo servoA;
Servo servoB;

TFT_eSPI tft;
// 8-bit depth keeps the sprite near 46KB, leaving internal RAM free for the
// WiFi + HomeKit stacks. This UI is flat colour, so the loss is not visible.
TFT_eSprite face(&tft);

constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 170;
constexpr int HEADER_H = 26;
constexpr int FACE_H = SCREEN_H - HEADER_H;

uint16_t C_RED, C_DARKRED, C_GOLD, C_AMBER, C_DIM;

enum State { CLOSED, OPENING, OPEN, CLOSING };

// 0.0 = fully closed, 1.0 = fully open.
float openness = 0.0f;
float moveFrom = 0.0f;
float moveTo = 0.0f;
uint32_t moveStart = 0;
uint32_t moveDuration = OPEN_TIME_MS;
bool moving = false;
bool desiredOpen = true;  // boots open
bool needsRedraw = true;
bool servosLive = false;
uint32_t relaxAt = 0;  // when non-zero, cut servo drive at this millis().

// Post-close sequence: 0 = idle, 1 = pressing into the stop, 2 = backing off.
uint8_t closePhase = 0;
uint32_t seatUntil = 0;

// Degrees expressed on the 0..1 openness axis.
constexpr float TRAVEL_SPAN_DEG = SERVO_A_OPEN_DEG > SERVO_A_CLOSED_DEG
                                     ? (SERVO_A_OPEN_DEG - SERVO_A_CLOSED_DEG)
                                     : (SERVO_A_CLOSED_DEG - SERVO_A_OPEN_DEG);
constexpr float BACKOFF_OPENNESS = CLOSE_BACKOFF_DEG / TRAVEL_SPAN_DEG;
constexpr float KICK_OPENNESS = OPEN_KICK_DEG / TRAVEL_SPAN_DEG;
// Negative openness = past the shut position, i.e. pressing into the stop.
constexpr float SHUT_OPENNESS = -CLOSE_PRESS_DEG / TRAVEL_SPAN_DEG;

uint32_t kickUntil = 0;  // hold the breakaway step until this millis().

// ---------------------------------------------------------------- motion

// write() clamps to 0-180, which is exactly where the closing press needs to go
// past. Convert to microseconds ourselves and clamp to the absolute limits.
static int degToUs(float deg) {
    float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f);
    return (int)constrain(lroundf(us), (long)SERVO_ABS_MIN_US, (long)SERVO_ABS_MAX_US);
}

static void applyServos() {
    if (!servosLive) return;
    float a = SERVO_A_CLOSED_DEG + (SERVO_A_OPEN_DEG - SERVO_A_CLOSED_DEG) * openness;
    float b = SERVO_B_CLOSED_DEG + (SERVO_B_OPEN_DEG - SERVO_B_CLOSED_DEG) * openness;
    servoA.writeMicroseconds(degToUs(a));
    servoB.writeMicroseconds(degToUs(b));
}

// Re-arms the PWM and immediately restates the current pose, so the servos
// resume from where the plate actually is instead of jumping to a stale angle.
static void wakeServos() {
    if (servosLive) return;
    servoA.setPeriodHertz(SERVO_FREQ_HZ);
    servoB.setPeriodHertz(SERVO_FREQ_HZ);
    servoA.attach(PIN_SERVO_A, SERVO_MIN_US, SERVO_MAX_US);
    servoB.attach(PIN_SERVO_B, SERVO_MIN_US, SERVO_MAX_US);
    servosLive = true;
    applyServos();
}

// Stops the pulse train. The servos go limp and stop drawing holding current.
static void relaxServos() {
    if (!servosLive) return;
    servoA.detach();
    servoB.detach();
    servosLive = false;
    relaxAt = 0;
    needsRedraw = true;
}

// Ramps openness to a raw target. duration 0 means "scale the direction's time".
static bool startMoveTo(float target, uint32_t duration) {
    wakeServos();
    if (fabsf(target - openness) < 0.001f) {
        moving = false;
        return false;
    }
    moveFrom = openness;
    moveTo = target;
    moveStart = millis();
    uint32_t fullSweep = (moveTo > moveFrom) ? OPEN_TIME_MS : CLOSE_TIME_MS;
    moveDuration = duration ? duration : (uint32_t)(fabsf(moveTo - moveFrom) * fullSweep);
    if (moveDuration < 50) moveDuration = 50;
    // Only lifting off the stop needs the breakaway step.
    kickUntil = 0;
    if (OPEN_KICK_DEG > 0.0f && moveTo > moveFrom && moveFrom <= 0.02f) {
        kickUntil = moveStart + OPEN_KICK_MS;
    }
    moving = true;
    needsRedraw = true;
    return true;
}

// Starts a move toward open/closed. Safe to call mid-move: it picks up from the
// current position, so a reversal from Siri or the button takes effect at once.
static void requestOpen(bool open) {
    desiredOpen = open;
    relaxAt = 0;
    closePhase = 0;
    if (!startMoveTo(open ? 1.0f : SHUT_OPENNESS, 0)) {
        // Already there: if that pose is closed, let it relax again.
        if (RELAX_WHEN_CLOSED && !open) relaxAt = millis() + RELAX_DELAY_MS;
        needsRedraw = true;
    }
}

// ---------------------------------------------------------------- HomeKit

struct FaceplateSwitch : Service::Switch {
    SpanCharacteristic *power;

    // Matches the boot pose so the Home app is not out of step on power-up.
    FaceplateSwitch() : Service::Switch() { power = new Characteristic::On(true); }

    boolean update() override {
        requestOpen(power->getNewVal());
        return true;
    }
};

FaceplateSwitch *hkSwitch = nullptr;

// ------------------------------------------------------------ calibration

// '@d <deg>' parks the pair at the shut pose, <deg> past the nominal closed
// angle, and holds it so the real seated position can be found by eye.
// Positive values go further shut (negative servo angles on B).
static void cmdShutDepth(const char *buf) {
    float d = atof(buf + 1);
    relaxAt = 0;
    closePhase = 0;
    kickUntil = 0;
    moving = false;
    openness = -d / TRAVEL_SPAN_DEG;
    wakeServos();
    applyServos();
    float a = SERVO_A_CLOSED_DEG + (SERVO_A_OPEN_DEG - SERVO_A_CLOSED_DEG) * openness;
    float b = SERVO_B_CLOSED_DEG + (SERVO_B_OPEN_DEG - SERVO_B_CLOSED_DEG) * openness;
    int ua = degToUs(a), ub = degToUs(b);
    Serial.printf("\nShut depth %.1f deg:  A = %.1f deg (%d us)   B = %.1f deg (%d us)\n", d, a, ua,
                  b, ub);
    if (ua >= SERVO_ABS_MAX_US || ub <= SERVO_ABS_MIN_US) {
        Serial.printf("*** CLAMPED at the pulse limit - the two sides are no longer symmetric.\n"
                      "*** Raise SERVO_US_HEADROOM (now %d us) before going deeper.\n",
                      SERVO_US_HEADROOM);
    }
    needsRedraw = true;
}

// '@x' cuts the drive immediately, so a stalled pose can be released fast.
static void cmdRelease(const char *buf) {
    relaxAt = 0;
    moving = false;
    relaxServos();
    Serial.printf("\nServos released.\n");
}

// ---------------------------------------------------------------- display

static void initColors() {
    C_RED = tft.color565(198, 24, 30);
    C_DARKRED = tft.color565(52, 6, 8);
    C_GOLD = tft.color565(255, 190, 60);
    C_AMBER = tft.color565(255, 130, 0);
    C_DIM = tft.color565(70, 70, 70);
}

static void drawHeader(bool wifiUp) {
    tft.fillRect(0, 0, SCREEN_W, HEADER_H, C_DARKRED);
    tft.drawFastHLine(0, HEADER_H - 1, SCREEN_W, C_GOLD);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(C_GOLD, C_DARKRED);
    tft.drawString("IRON MAN HELMET", 8, 5, 2);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(wifiUp ? C_GOLD : C_DIM, C_DARKRED);
    tft.drawString(wifiUp ? "HOMEKIT" : "NO WIFI", 312, 5, 2);
    tft.setTextDatum(TL_DATUM);
}

// Blend from the dim (closed) tone up to full gold as the faceplate opens.
static uint16_t eyeColor(float v) {
    return tft.color565((uint8_t)(40 + 215 * v), (uint8_t)(40 + 150 * v), (uint8_t)(40 + 20 * v));
}

// One angled eye slit; mirrored flips the slant for the other side.
static void drawEye(int x, int y, int w, int h, bool mirrored, uint16_t color) {
    int lean = w / 4;
    int x1 = mirrored ? x : x + lean;
    int x2 = mirrored ? x + w - lean : x + w;
    int x3 = mirrored ? x + w : x + w - lean;
    int x4 = mirrored ? x + lean : x;
    face.fillTriangle(x1, y, x2, y, x3, y + h, color);
    face.fillTriangle(x1, y, x3, y + h, x4, y + h, color);
}

static const char *stateLabel(State s) {
    switch (s) {
        case CLOSED: return "CLOSED";
        case OPENING: return "OPENING";
        case OPEN: return "OPEN";
        default: return "CLOSING";
    }
}

static uint16_t stateColor(State s) {
    switch (s) {
        case CLOSED: return C_RED;
        case OPEN: return C_GOLD;
        default: return C_AMBER;
    }
}

static void drawFace(State state) {
    face.fillSprite(TFT_BLACK);

    // The back-off leaves a few degrees on the dial; report the closed pose as 0%.
    float shown = (state == CLOSED && openness <= BACKOFF_OPENNESS + 0.01f) ? 0.0f : openness;

    uint16_t eye = eyeColor(shown);
    drawEye(92, 6, 58, 18, false, eye);
    drawEye(170, 6, 58, 18, true, eye);

    face.setTextDatum(MC_DATUM);
    face.setTextColor(stateColor(state), TFT_BLACK);
    face.setTextSize(2);
    face.drawString(stateLabel(state), SCREEN_W / 2, 52, 4);
    face.setTextSize(1);

    constexpr int barX = 20, barY = 86, barW = 280, barH = 16;
    face.drawRect(barX, barY, barW, barH, C_DIM);
    int fill = (int)lroundf(constrain(shown, 0.0f, 1.0f) * (barW - 4));
    face.fillRect(barX + 2, barY + 2, fill, barH - 4, stateColor(state));

    char buf[24];
    snprintf(buf, sizeof(buf), "%d%%", (int)lroundf(shown * 100));
    face.setTextDatum(TL_DATUM);
    face.setTextColor(C_DIM, TFT_BLACK);
    face.drawString(buf, 20, 116, 2);

    face.setTextDatum(TR_DATUM);
    if (moving) {
        face.setTextColor(C_AMBER, TFT_BLACK);
        face.drawString("MOVING", 300, 116, 2);
    } else if (!servosLive) {
        face.setTextColor(C_DIM, TFT_BLACK);
        face.drawString("SERVOS OFF", 300, 116, 2);
    } else {
        face.setTextColor(C_DIM, TFT_BLACK);
        face.drawString("BTN OR SIRI", 300, 116, 2);
    }
    face.setTextDatum(TL_DATUM);

    face.pushSprite(0, HEADER_H);
}

// ---------------------------------------------------------------- setup

void setup() {
    pinMode(PIN_PWR_ON, OUTPUT);
    digitalWrite(PIN_PWR_ON, HIGH);

    pinMode(PIN_BTN_BOOT, INPUT_PULLUP);
    pinMode(PIN_BTN_14, INPUT_PULLUP);

    // HomeSpan does not open the port itself; its serial CLI (WiFi setup, pairing
    // info) is dead without this. Give USB CDC a moment to enumerate first.
    Serial.begin(115200);
    delay(500);

    tft.init();
    tft.setRotation(1);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    initColors();
    tft.fillScreen(TFT_BLACK);
    drawHeader(false);

    face.setColorDepth(8);
    face.createSprite(SCREEN_W, FACE_H);

    // ESP32Servo needs explicit LEDC timer allocation before attach().
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    // Boot into the open pose, so the first press or Siri command closes. The
    // servos stay powered here: open is held against gravity, not on a stop.
    openness = 1.0f;
    wakeServos();
    drawFace(OPEN);

    homeSpan.setPairingCode("46637726");
    homeSpan.setQRID("HELM");
    homeSpan.begin(Category::Switches, "Iron Man Helmet");

    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Iron Man Helmet");
    new Characteristic::Manufacturer("DIY");
    new Characteristic::Model("MK I");
    hkSwitch = new FaceplateSwitch();

    new SpanUserCommand('d', "<deg> - hold the shut pose <deg> past nominal closed", cmdShutDepth);
    new SpanUserCommand('x', "- release the servos now", cmdRelease);
}

// ---------------------------------------------------------------- loop

void loop() {
    homeSpan.poll();

    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    if (now - lastUpdate < UPDATE_INTERVAL_MS) return;
    lastUpdate = now;

    static bool prevPressed = false;
    static uint32_t lastEdge = 0;
    static bool prevWifi = false;

    // Either button counts; both read LOW when pressed.
    bool pressed = (digitalRead(PIN_BTN_BOOT) == LOW) || (digitalRead(PIN_BTN_14) == LOW);
    if (pressed != prevPressed && (now - lastEdge) > DEBOUNCE_MS) {
        lastEdge = now;
        prevPressed = pressed;
        if (pressed) {
            requestOpen(!desiredOpen);
            // Push the new state to HomeKit so the Home app tracks the button.
            if (hkSwitch) hkSwitch->power->setVal(desiredOpen);
        }
    }

    if (moving) {
        uint32_t elapsed = now - moveStart;
        float t = (elapsed >= moveDuration) ? 1.0f : (float)elapsed / (float)moveDuration;
        openness = moveFrom + (moveTo - moveFrom) * t;
        // Floor the command at the kick angle until the ramp overtakes it, so
        // the servo starts against a large error instead of creeping.
        if (kickUntil != 0) {
            if (now < kickUntil && openness < KICK_OPENNESS) {
                openness = KICK_OPENNESS;
            } else if (now >= kickUntil) {
                kickUntil = 0;
            }
        }
        if (t >= 1.0f) {
            openness = moveTo;
            moving = false;
            if (closePhase == 2) {
                // Pressure released; safe to cut drive.
                closePhase = 0;
                if (RELAX_WHEN_CLOSED) relaxAt = now + RELAX_DELAY_MS;
            } else if (openness <= 0.001f) {
                // Just reached the stop. Press into it, then unload.
                if (CLOSE_BACKOFF_DEG > 0.0f) {
                    closePhase = 1;
                    seatUntil = now + SEAT_TIME_MS;
                } else if (RELAX_WHEN_CLOSED) {
                    relaxAt = now + RELAX_DELAY_MS;
                }
            }
        }
        applyServos();
        needsRedraw = true;
    }

    if (!moving && closePhase == 1 && now >= seatUntil) {
        closePhase = 2;
        startMoveTo(BACKOFF_OPENNESS, BACKOFF_TIME_MS);
    }

    if (relaxAt != 0 && now >= relaxAt) relaxServos();

    bool wifiUp = (WiFi.status() == WL_CONNECTED);
    if (wifiUp != prevWifi) {
        prevWifi = wifiUp;
        drawHeader(wifiUp);
    }

    if (needsRedraw) {
        needsRedraw = false;
        State st;
        if (closePhase != 0) {
            st = CLOSED;  // seating and backing off are still "closed"
        } else if (moving) {
            st = (moveTo > moveFrom) ? OPENING : CLOSING;
        } else {
            st = (openness > 0.5f) ? OPEN : CLOSED;
        }
        drawFace(st);
    }
}
