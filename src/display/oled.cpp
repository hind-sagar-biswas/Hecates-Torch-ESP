#include "oled.h"
#include "config.h"
#include "routing/neighbor_table.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// 128×64 OLED. Layout:
//   Left panel  (x: 0–74)  — text: mood, SOS, cost, direction label, exit
//   Divider     (x: 75)    — vertical line
//   Right panel (x: 76–127) — direction arrow (8-directional)
//
// Direction arrow is computed geometrically:
//   dx = neighbor.x - my.x,  dy = neighbor.y - my.y
//   angle = atan2(dy, dx) → mapped to 8 compass sectors
// We draw a simple arrow from center of right panel toward that sector.

#define SCREEN_W 128
#define SCREEN_H 64
#define DIVIDER_X 75

// Right panel arrow center
#define ARROW_CX 101 // (75 + 128) / 2
#define ARROW_CY 32  // 64 / 2
#define ARROW_LEN 18 // pixels from center to tip

static Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
static bool oled_ok = false;

// ─── Direction helpers ───────────────────────────────────────────────────────

// 8-direction enum for internal use
enum class Dir8
{
    E,
    NE,
    N,
    NW,
    W,
    SW,
    S,
    SE,
    NONE
};

static Dir8 angleToDir8(float dx, float dy)
{
    // atan2 returns angle in radians, -π to π, measured from +x axis
    // We flip dy sign because screen y increases downward but building
    // y coordinates increase upward (standard floor plan convention).
    float angle = atan2(-dy, dx); // radians

    // Normalize to 0–2π
    if (angle < 0)
        angle += 2.0f * M_PI;

    // 8 sectors of 45° each, offset by 22.5° so E is centered on 0°
    int sector = (int)((angle + M_PI / 8.0f) / (M_PI / 4.0f)) % 8;

    // sector 0=E, 1=NE, 2=N, 3=NW, 4=W, 5=SW, 6=S, 7=SE
    return static_cast<Dir8>(sector);
}

static const char *dir8Label(Dir8 d)
{
    switch (d)
    {
    case Dir8::E:
        return "RIGHT";
    case Dir8::NE:
        return "UP-RIGHT";
    case Dir8::N:
        return "UP";
    case Dir8::NW:
        return "UP-LEFT";
    case Dir8::W:
        return "LEFT";
    case Dir8::SW:
        return "DN-LEFT";
    case Dir8::S:
        return "DOWN";
    case Dir8::SE:
        return "DN-RIGHT";
    default:
        return "---";
    }
}

// Draw a line-based arrow from center toward the given direction
static void drawArrow(Dir8 d)
{
    if (d == Dir8::NONE)
        return;

    // Unit vector for each direction (screen coords: right=+x, down=+y)
    float vx = 0, vy = 0;
    switch (d)
    {
    case Dir8::E:
        vx = 1.0f;
        vy = 0.0f;
        break;
    case Dir8::NE:
        vx = 0.7f;
        vy = -0.7f;
        break;
    case Dir8::N:
        vx = 0.0f;
        vy = -1.0f;
        break;
    case Dir8::NW:
        vx = -0.7f;
        vy = -0.7f;
        break;
    case Dir8::W:
        vx = -1.0f;
        vy = 0.0f;
        break;
    case Dir8::SW:
        vx = -0.7f;
        vy = 0.7f;
        break;
    case Dir8::S:
        vx = 0.0f;
        vy = 1.0f;
        break;
    case Dir8::SE:
        vx = 0.7f;
        vy = 0.7f;
        break;
    default:
        return;
    }

    int tx = ARROW_CX + (int)(vx * ARROW_LEN);
    int ty = ARROW_CY + (int)(vy * ARROW_LEN);

    // Shaft
    display.drawLine(ARROW_CX, ARROW_CY, tx, ty, SSD1306_WHITE);

    // Arrowhead — two short lines at ~135° from shaft direction
    float headLen = 6.0f;
    float ax = vy * headLen; // perpendicular component
    float ay = -vx * headLen;

    // Left barb
    display.drawLine(tx, ty,
                     tx + (int)(-vx * headLen * 0.7f + ax * 0.5f),
                     ty + (int)(-vy * headLen * 0.7f + ay * 0.5f),
                     SSD1306_WHITE);
    // Right barb
    display.drawLine(tx, ty,
                     tx + (int)(-vx * headLen * 0.7f - ax * 0.5f),
                     ty + (int)(-vy * headLen * 0.7f - ay * 0.5f),
                     SSD1306_WHITE);
}

// ─── Public ──────────────────────────────────────────────────────────────────

void oledInit()
{
    oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (!oled_ok)
    {
        Serial.println("OLED init failed — display disabled.");
        return;
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
}

void oledUpdate(const NodeState &node)
{
    if (!oled_ok)
        return;
    display.clearDisplay();

    // ── Divider line ─────────────────────────────────────────────────────────
    display.drawLine(DIVIDER_X, 0, DIVIDER_X, SCREEN_H - 1, SSD1306_WHITE);

    // ── Left panel — text ─────────────────────────────────────────────────────
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Line 0: Mood
    const char *moodStr = "STBY";
    switch (node.mood)
    {
    case Mood::WARN:
        moodStr = "WARN";
        break;
    case Mood::ALERT:
        moodStr = "ALERT";
        break;
    case Mood::EVAC:
        moodStr = "EVAC";
        break;
    case Mood::RESCUE:
        moodStr = "RESCUE";
        break;
    default:
        break;
    }
    display.setCursor(0, 0);
    display.print("Mood: ");
    display.print(moodStr);

    // Line 1: SOS
    display.setCursor(0, 11);
    display.print("SOS : ");
    if (node.sos_active)
    {
        display.print("N");
        display.print(node.node_id);
    }
    else
    {
        display.print("NONE");
    }

    // Line 2: Cost
    display.setCursor(0, 22);
    display.print("Cost: ");
    if (node.best_exit_cost >= 99999.0f)
    {
        display.print("INF");
    }
    else
    {
        display.print(node.best_exit_cost, 2);
    }

    // Line 3: Direction label
    // Resolve direction node_id → x,y → angle → label
    Dir8 dir = Dir8::NONE;
    const char *dirLabel = "---";

    if (node.direction == 255)
    { // DIRECTION_SELF
        dirLabel = "EXIT";
    }
    else if (node.direction != 254)
    { // DIRECTION_NONE = 254
        NeighborEntry *nb = const_cast<NodeState &>(node).neighbors.find(node.direction);
        if (nb)
        {
            float dx = nb->x - node.x;
            float dy = nb->y - node.y;
            dir = angleToDir8(dx, dy);
            dirLabel = dir8Label(dir);
        }
    }

    display.setCursor(0, 33);
    display.print("Dir : ");
    display.print(dirLabel);

    // Line 4: Exit status
    display.setCursor(0, 44);
    display.print("Exit: ");
    display.print(node.is_exit ? "YES" : "NO");

    // ── Right panel — arrow ───────────────────────────────────────────────────
    if (node.direction == 255)
    {
        // This node IS the exit — draw a filled circle as "you are here"
        display.fillCircle(ARROW_CX, ARROW_CY, 6, SSD1306_WHITE);
    }
    else
    {
        drawArrow(dir);
    }

    display.display();
}