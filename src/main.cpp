#include "credentials.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TFT_eSPI.h>

#include <WiFi.h>
#include <time.h>

#include "../lib/sgp4/TLE.h"
#include "../lib/sgp4/SGP4.h"
#include "../lib/libpredict-2.0.0/src/unsorted.h"
#include "../lib/libpredict-2.0.0/include/predict/predict.h"

void printMemoryStatus(const char* stepName) {
    struct mallinfo mi = mallinfo();
    size_t free_heap = mi.fordblks;
    size_t min_free = rp2040.getFreeHeap();
    Serial1.printf("[MEMORY] %s:\tfree heap (mallinfo): %d bytes\tfree heap (rp2040): %d bytes\n",
        stepName,
        free_heap,
        min_free
        );
}

const char *ssid = STASSID;
const char *password = STAPSK;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);

enum HudStyle : uint8_t {
    HUD_CROSSHAIR_SIMPLE = 0,
    HUD_CROSSHAIR_DIAGONAL,
    HUD_CROSSHAIR_ARROWS,
    HUD_CROSSHAIR_ARROWS_H,
    HUD_TARGET_BOX,
    HUD_CIRCLE,
};

struct OrbitData {
    double mean_motion;
    double eccentricity;
    double inclination;
    double raan;
    double arg_pericenter;
    double mean_anomaly;
    double bstar;
    double mean_dot;
    double mean_ddot;
    double epoch;
    int epoch_year;
    /* // these fields take up space unnecessarily
    int epoch_month;
    int epoch_day;
    int epoch_hour;
    int epoch_minute;
    double epoch_second;
    */
};

// Constellation chosen for having relatively few satellites
const char* const EXAMPLE_TLES[] = {
// OBJECT_NAME,OBJECT_ID,EPOCH,MEAN_MOTION,ECCENTRICITY,INCLINATION,RA_OF_ASC_NODE,ARG_OF_PERICENTER,MEAN_ANOMALY,EPHEMERIS_TYPE,CLASSIFICATION_TYPE,NORAD_CAT_ID,ELEMENT_SET_NO,REV_AT_EPOCH,BSTAR,MEAN_MOTION_DOT,MEAN_MOTION_DDOT
    "GPS BIIR-5  (PRN 22),2000-040A,2026-07-15T16:33:36.595296,2.00557849,.01205037,54.8484,214.0300,302.7446,41.0255,0,U,26407,999,19051,0,.47E-6,0",
    "GPS BIIR-8  (PRN 16),2003-005A,2026-07-16T00:55:17.989824,2.00558980,.01482928,54.8788,213.8224,53.1718,318.3547,0,U,27663,999,17189,0,.44E-6,0",
    "GPS BIIR-11 (PRN 19),2004-009A,2026-07-15T07:44:57.281280,2.00553680,.01171901,54.8064,274.5493,174.3466,193.0919,0,U,28190,999,16351,0,-.17E-6,0",
    "GPS BIIR-13 (PRN 02),2004-045A,2026-07-15T06:03:08.669088,2.00557037,.01701848,55.1181,323.7660,317.0606,64.4184,0,U,28474,999,15899,0,-.92E-6,0",
    "GPS BIIRM-1 (PRN 17),2005-038A,2026-07-16T06:40:53.963904,2.00563624,.01282296,54.8546,272.0021,296.4273,70.6506,0,U,28874,999,15242,0,-.22E-6,0",
    "GPS BIIRM-2 (PRN 31),2006-042A,2026-07-15T22:14:42.476928,2.00568164,.0107212,54.7129,150.1979,55.1773,317.0416,0,U,29486,999,14497,0,.16E-6,0",
    "GPS BIIRM-3 (PRN 12),2006-052A,2026-07-15T20:06:41.605056,2.00559692,.008474,54.9322,212.8704,91.0004,270.0084,0,U,29601,999,14398,0,.46E-6,0",
    "GPS BIIRM-4 (PRN 15),2007-047A,2026-07-14T15:32:58.928064,2.00553904,.0167775,54.1153,78.9337,87.9759,274.0038,0,U,32260,999,13736,0,-.33E-6,0",
    "GPS BIIRM-5 (PRN 29),2007-062A,2026-07-14T22:49:13.318176,2.00561560,.0036056,55.0097,273.0354,167.4938,179.2057,0,U,32384,999,13606,0,-.11E-6,0",
    "GPS BIIRM-6 (PRN 07),2008-012A,2026-07-16T01:15:16.151328,2.00573136,.0211681,54.5002,148.7716,248.1586,109.5403,0,U,32711,999,13433,0,.15E-6,0",
    "GPS BIIRM-8 (PRN 05),2009-043A,2026-07-14T13:26:31.947072,2.00552733,.005373,56.2274,26.3776,84.3592,285.8100,0,U,35752,999,12391,0,-.68E-6,0",
    "GPS BIIF-1  (PRN 25),2010-022A,2026-07-15T08:58:33.645792,2.00563746,.01267238,54.2685,207.4265,66.1541,291.5416,0,U,36585,999,11815,0,.49E-6,0",
    "GPS BIIF-3  (PRN 24),2012-053A,2026-07-14T17:41:43.071648,2.00567894,.01819929,53.5896,142.4706,66.2134,295.6400,0,U,38833,999,9999,0,.2E-7,0",
    "GPS BIIF-4  (PRN 27),2013-023A,2026-07-15T14:27:10.507680,2.00563312,.0141388,54.4824,267.8149,51.1134,310.2184,0,U,39166,999,9645,0,-.9E-7,0",
    "GPS BIIF-5  (PRN 30),2014-008A,2026-07-15T14:29:46.846752,2.00561577,.0084091,53.6578,148.1607,231.5484,127.6542,0,U,39533,999,9025,0,.11E-6,0",
    "GPS BIIF-6  (PRN 06),2014-026A,2026-07-15T09:09:21.973248,2.00558840,.00381411,56.4831,330.4202,325.7796,33.9711,0,U,39741,999,8910,0,-.95E-6,0",
    "GPS BIIF-7  (PRN 09),2014-045A,2026-07-14T23:23:28.311936,2.00560212,.0034406,55.3710,87.3578,118.3743,242.0239,0,U,40105,999,8663,0,-.25E-6,0",
    "GPS BIIF-8  (PRN 03),2014-068A,2026-07-14T20:13:45.283872,2.00553162,.00674067,57.0066,29.8902,72.2885,288.4955,0,U,40294,999,8576,0,-.65E-6,0",
    "GPS BIIF-9  (PRN 26),2015-013A,2026-07-15T23:33:30.931200,2.00549601,.0110386,53.1660,202.9866,40.3901,320.4392,0,U,40534,999,8240,0,.42E-6,0",
    "GPS BIIF-10 (PRN 08),2015-033A,2026-07-15T03:25:15.578688,2.00554681,.0115357,53.9612,265.9327,30.2860,330.4577,0,U,40730,999,8055,0,-.2E-7,0",
    "GPS BIIF-11 (PRN 10),2015-062A,2026-07-13T04:52:25.143456,2.00566897,.01127742,56.9770,29.8096,230.8884,128.1532,0,U,41019,999,7835,0,-.56E-6,0",
    "GPS BIIF-12 (PRN 32),2016-007A,2026-07-15T18:53:39.029856,2.00575339,.00966286,55.5414,88.3254,247.6143,111.4117,0,U,41328,999,7641,0,-.24E-6,0",
    "GPS BIII-1  (PRN 04),2018-109A,2026-07-16T04:07:31.579392,2.00562827,.00385958,55.6993,91.0428,197.1262,334.9231,0,U,43873,999,5565,0,-.21E-6,0",
    "GPS BIII-2  (PRN 18),2019-056A,2026-07-15T19:19:31.702656,2.00569977,.00634641,55.6216,330.0052,198.1718,333.4897,0,U,44506,999,5064,0,-.96E-6,0",
    "GPS BIII-3  (PRN 23),2020-041A,2026-07-15T21:42:14.592384,2.00557785,.0065213,56.6887,27.8299,204.6780,335.6765,0,U,45854,999,4461,0,-.75E-6,0",
    "GPS BIII-4  (PRN 14),2020-078A,2026-07-15T16:30:38.802240,2.00564949,.00745628,53.9772,209.6224,207.8336,153.6237,0,U,46826,999,4208,0,.46E-6,0",
    "GPS BIII-5  (PRN 11),2021-054A,2026-07-15T22:15:10.747872,2.00566994,.0026474,55.1591,331.3041,228.5431,132.0309,0,U,48859,999,3733,0,-.98E-6,0",
    "GPS BIII-6  (PRN 28),2023-009A,2026-07-16T03:01:57.879840,2.00569500,.00062594,55.1242,147.3813,321.9339,221.1207,0,U,55268,999,2582,0,.16E-6,0",
    "GPS BIII-7  (PRN 01),2024-242A,2026-07-16T06:14:57.087744,2.00572022,.00179284,54.8354,333.0025,4.3490,357.8288,0,U,62339,999,1184,0,-.1E-5,0",
    "GPS BIII-8  (PRN 21),2025-116A,2026-07-16T00:06:34.149024,2.00573669,.00084315,55.2543,29.9713,339.3849,21.2010,0,U,64202,999,845,0,-.76E-6,0",
    "GPS BIII-9  (PRN 20),2026-017A,2026-07-15T02:48:10.740672,2.00570507,.00216395,55.1061,87.6342,260.3058,99.5008,0,U,67588,999,43217,0,-.26E-6,0",
    "GPS BIII-10 (PRN 13),2026-087A,2026-07-15T11:14:09.755808,2.00572628,.00204615,54.9617,206.6692,106.0864,353.1427,0,U,68791,999,140,0,.49E-6,0",
};

OrbitData example_satellites[std::size(EXAMPLE_TLES)];

// TODO: get from GPS server
constexpr double my_latitude = 47.0;
constexpr double my_longitude = 19.0;
constexpr double my_altitude = 102.0;

uint8_t led_depth = 0;

constexpr uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | // Keep top 5 bits of Red, shift to bit 11-15
           ((g & 0xFC) << 3) | // Keep top 6 bits of Green, shift to bit 5-10
           ((b & 0xF8) >> 3);  // Keep top 5 bits of Blue, shift to bit 0-4
}

constexpr uint16_t HEX565(uint32_t hex) {
    return ((((hex >> 16) & 0xF8) << 8) | // Extract R (bits 16-23)
            (((hex >> 8)  & 0xFC) << 3) | // Extract G (bits 8-15)
            (((hex)       & 0xF8) >> 3)); // Extract B (bits 0-7)
}

void init_led() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void led_on() {
    int current_depth = __atomic_add_fetch(&led_depth, 1, __ATOMIC_SEQ_CST);
    if (current_depth == 1) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

void led_off() {
    int current_depth = __atomic_sub_fetch(&led_depth, 1, __ATOMIC_SEQ_CST);
    if (current_depth == 0) {
        digitalWrite(LED_BUILTIN, LOW);
    } else if (current_depth < 0) {
        __atomic_store_n(&led_depth, 0, __ATOMIC_SEQ_CST);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

void drawTick(float angle_degrees, uint8_t length, uint16_t color) {
    float angle_radians = angle_degrees * PI / 180.0;
    float sin_value = sinf(angle_radians);
    float cos_value = cosf(angle_radians);
    canvas.drawLine(
        round(sin_value * 120 + 120),
        round(cos_value * 120 + 120),
        round(sin_value * (120 - length) + 120),
        round(cos_value * (120 - length) + 120),
        color
    );
}

void drawGrid() {
    canvas.fillSprite(HEX565(0x000000));
    uint16_t grid_color_bg = HEX565(0x222222);
    uint16_t grid_color_fg = HEX565(0x333333);
    canvas.drawFastVLine(120, 0, 240, grid_color_bg);
    canvas.drawFastHLine(0, 120, 240, grid_color_bg);
    uint8_t rings = 3;
    for (int i = 1; i < rings; i++) {
        canvas.drawCircle(120, 120, (120 / rings - 1) * i, grid_color_bg);
    }
    for (uint16_t i = 0; i < 360; i += 5) {
        uint8_t length = 2;
        if (i % 30 == 0) {
            length = 10;
        }else if (i % 15 == 0) {
            length = 5;
        }
        drawTick(i, 2, grid_color_fg);
    }
    for (uint16_t i = 0; i < 360; i += 15) {
        drawTick(i, 5, grid_color_fg);
    }
    for (uint16_t i = 0; i < 360; i += 30) {
        drawTick(i, 10, grid_color_fg);
    }
}

#define ICON_RADIUS 10

// the screen is 240x240 pixels, 8 bits are enough
void drawIcon(uint8_t x, uint8_t y, uint32_t color, HudStyle style) {
    uint8_t radius = ICON_RADIUS;
    switch (style) {
        case HUD_CROSSHAIR_SIMPLE:
            canvas.drawFastVLine(x, y - radius, radius - 1, HEX565(color));
            canvas.drawFastVLine(x, y + 2,      radius - 1, HEX565(color));

            canvas.drawFastHLine(x - radius,     y, radius - 1, HEX565(color));
            canvas.drawFastHLine(x + 2,          y, radius - 1, HEX565(color));
            break;
        case HUD_CROSSHAIR_DIAGONAL:
            canvas.drawLine(x - radius, y - radius, x - 2, y - 2, HEX565(color));
            canvas.drawLine(x + radius, y - radius, x + 2, y - 2, HEX565(color));
            canvas.drawLine(x - radius, y + radius, x - 2, y + 2, HEX565(color));
            canvas.drawLine(x + radius, y + radius, x + 2, y + 2, HEX565(color));
            break;
        case HUD_CROSSHAIR_ARROWS_H:
            // center
            // canvas.drawPixel(x, y, HEX565(color));

            // arrows
            canvas.drawLine(x - (radius / 2), y - 1, x - radius, y - (radius / 3), HEX565(color));
            canvas.drawLine(x - (radius / 2), y + 1, x - radius, y + (radius / 3), HEX565(color));

            canvas.drawLine(x + (radius / 2), y - 1, x + radius, y - (radius / 3), HEX565(color));
            canvas.drawLine(x + (radius / 2), y + 1, x + radius, y + (radius / 3), HEX565(color));
            break;
        case HUD_CROSSHAIR_ARROWS:
            // center
            // canvas.drawPixel(x, y, HEX565(color));

            // arrows
            canvas.drawLine(x - (radius / 2), y - 1, x - radius, y - (radius / 3), HEX565(color));
            canvas.drawLine(x - (radius / 2), y + 1, x - radius, y + (radius / 3), HEX565(color));

            canvas.drawLine(x + (radius / 2), y - 1, x + radius, y - (radius / 3), HEX565(color));
            canvas.drawLine(x + (radius / 2), y + 1, x + radius, y + (radius / 3), HEX565(color));

            canvas.drawLine(x - 1, y - (radius / 2), x - (radius / 3), y - radius, HEX565(color));
            canvas.drawLine(x - 1, y + (radius / 2), x - (radius / 3), y + radius, HEX565(color));

            canvas.drawLine(x + 1, y - (radius / 2), x + (radius / 3), y - radius, HEX565(color));
            canvas.drawLine(x + 1, y + (radius / 2), x + (radius / 3), y + radius, HEX565(color));
            break;
        case HUD_TARGET_BOX:
            // outer frame
            canvas.drawLine(x - radius, y - 2, x - 2, y - radius, HEX565(color));
            canvas.drawLine(x - radius, y + 2, x - 2, y + radius, HEX565(color));

            canvas.drawLine(x + radius, y + 2, x + 2, y + radius, HEX565(color));
            canvas.drawLine(x + radius, y - 2, x + 2, y - radius, HEX565(color));

            // center cross
            // canvas.drawFastHLine(x - 1, y, 3, HEX565(color));
            // canvas.drawFastVLine(x, y - 1, 3, HEX565(color));
            // center dot
            canvas.drawPixel(x, y, HEX565(color));
            break;
        case HUD_CIRCLE:
            // outer frame
            canvas.drawCircle(x, y, radius, HEX565(color));
            break;
    }
}

void takeScreenshot() {
    while (!Serial) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    uint16_t* fb = (uint16_t*)canvas.getPointer();
    if (!fb) {
        Serial1.println("Error: Framebuffer pointer is NULL");
        return;
    }
    Serial.print("\033c");
    for (int y = 0; y < 240; y+= 2) {
        for (int x = 0; x < 240; x++) {
            uint16_t raw_pixel1 = fb[y * 240 + x];
            // swap endianness
            uint16_t pixel1 = (raw_pixel1 >> 8) | (raw_pixel1 << 8);

            uint16_t raw_pixel2 = fb[y * 240 + x + 240];
            // swap endianness
            uint16_t pixel2 = (raw_pixel2 >> 8) | (raw_pixel2 << 8);

            uint8_t r1 = ((pixel1 >> 11) & 0x1F) * 255 / 31;
            uint8_t g1 = ((pixel1 >> 5)  & 0x3F) * 255 / 63;
            uint8_t b1 = (pixel1         & 0x1F) * 255 / 31;

            uint8_t r2 = ((pixel2 >> 11) & 0x1F) * 255 / 31;
            uint8_t g2 = ((pixel2 >> 5)  & 0x3F) * 255 / 63;
            uint8_t b2 = (pixel2         & 0x1F) * 255 / 31;
            Serial.printf("\033[38;2;%d;%d;%dm\033[48;2;%d;%d;%dm\u2580", r1, g1, b1, r2, g2, b2);
        }
        Serial.print("\033[0m\r\n");
    }
}

struct AzEl{
    double azimuth;
    double elevation;
};

struct ScreenXY{
    double x;
    double y;
};

AzEl tle_to_azel() {
    AzEl this_azel = {
        0.0,
        0.0
    };
    return this_azel;
}

ScreenXY azel_to_xy(AzEl azel) {
    ScreenXY this_xy = {};
    double sin_azimuth = sin(azel.azimuth * DEG_TO_RAD);
    double cos_azimuth = cos(azel.azimuth * DEG_TO_RAD);

    this_xy.x = round((sin_azimuth * (90 - azel.elevation) * (120.0 / 90.0)) + 120);
    this_xy.y = round((-cos_azimuth * (90 - azel.elevation) * (120.0 / 90.0)) + 120);

    return this_xy;
}

class CsvRowParser {
private:
    const char* cursor;

public:
    CsvRowParser(const char* row) : cursor(row) {}

    // Extracts the next column into a stack buffer and slides the cursor.
    // If dest is nullptr, it safely skips the column entirely.
    bool nextColumn(char* dest, size_t maxLen) {
        if (!cursor || *cursor == '\0') {
            return false;
        }

        size_t len = 0;
        while (*cursor != '\0' && *cursor != ',' && *cursor != '\n' && *cursor != '\r') {
            if (dest && len < maxLen - 1) {
                dest[len++] = *cursor;
            }
            cursor++;
        }

        if (dest && maxLen > 0) {
            dest[len] = '\0';
        }

        // Skip the comma to align with the next column
        if (*cursor == ',') {
            cursor++;
        }
        return true;
    }
};

bool hasTime = false;
TaskHandle_t xTimeSyncTaskHandle;
[[noreturn]] void timeSyncTask(void *pvParameters) {
    while (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    NTP.begin("192.168.100.88");
    Serial1.println("Waiting for NTP sync");
    NTP.waitSet([]() {
        vTaskDelay(pdMS_TO_TICKS(100));
    });
    hasTime = true;
    Serial1.println("Time Synced from NTP!");
    while (true) {
        time_t now = time(nullptr);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        Serial1.println(asctime(&timeinfo));
        vTaskDelay(pdMS_TO_TICKS(10 * 60 * 1000));
    }
}

void drawIconAzEl(double az, double el, uint32_t color, HudStyle style) {
    AzEl azel = {
        .azimuth = az,
        .elevation = el
    };
    ScreenXY screenxy = azel_to_xy(azel);
    drawIcon(screenxy.x, screenxy.y, color, style);
}

double getCurrentJulianDate() {
    time_t now = time(nullptr); // Raw UTC Unix time
    return 2440587.5 + ((double)now / 86400.0);
}

double csvEpochToJulianDate(int year, int month, int day, int hour, int minute, double second) {
    // Treat January and February as months 13 and 14 of the previous year
    if (month <= 2) {
        year -= 1;
        month += 12;
    }

    // Gregorian Calendar adjustments
    int A = year / 100;
    int B = 2 - A + (A / 4);

    // Precise day fraction
    double day_fraction = day + (hour / 24.0) + (minute / 1440.0) + (second / 86400.0);

    // Meeus Julian Date formula
    double jd = (int)(365.25 * (year + 4716)) +
                (int)(30.6001 * (month + 1)) +
                day_fraction + B - 1524.5;

    return jd;
}

constexpr double REVS_DAY_TO_RAD_MIN = (2 * PI) / 1440.0;

OrbitData parse_csv_gp(const char* csv) {
    CsvRowParser parser(csv);
    char buf[32];
    TLE sat;
    char satName[24] = {0};
    char epochStr[28] = {0};
    double epochJd = 0.0;
    // Column 0: OBJECT_NAME (string)
    parser.nextColumn(satName, sizeof(satName));

    // Column 1: OBJECT_ID (string) - skip (not needed for propagation)
    parser.nextColumn(nullptr, 0);

    // Column 2: EPOCH (ISO Timestamp: "2026-07-14T23:01:59.000000")
    parser.nextColumn(epochStr, sizeof(epochStr));

    int year = 0, month = 0, day = 0, hour = 0, minute = 0;
    double second = 0.0;
    if (sscanf(epochStr, "%d-%d-%dT%d:%d:%lf", &year, &month, &day, &hour, &minute, &second) == 6) {
        epochJd = csvEpochToJulianDate(year, month, day, hour, minute, second);
    } else {
        Serial1.print("Failed to parse ISO Epoch timestamp for ");
        Serial1.print(satName);
        Serial1.println("!");
    }

    // Column 3: MEAN_MOTION (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.n = strtod(buf, nullptr);

    // Column 4: ECCENTRICITY (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.ecc = strtod(buf, nullptr);

    // Column 5: INCLINATION (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.incDeg = strtod(buf, nullptr);

    // Column 6: RA_OF_ASC_NODE (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.raanDeg = strtod(buf, nullptr);

    // Column 7: ARG_OF_PERICENTER (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.argpDeg = strtod(buf, nullptr);

    // Column 8: MEAN_ANOMALY (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.maDeg = strtod(buf, nullptr);

    // Column 9: EPHEMERIS_TYPE - skip
    parser.nextColumn(nullptr, 0);

    // Column 10: CLASSIFICATION_TYPE - skip
    parser.nextColumn(nullptr, 0);

    // Column 11: NORAD_CAT_ID - skip
    parser.nextColumn(nullptr, 0);

    // Column 11: ELEMENT_SE_NO - skip
    parser.nextColumn(nullptr, 0);

    // Column 11: REV_AT_EPOCH - skip (not needed for propagation)
    parser.nextColumn(nullptr, 0);

    // Column 11: BSTAR (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.bstar = strtod(buf, nullptr);

    // Column 11: MEAN_MOTION_DOT (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.ndot = strtod(buf, nullptr);

    // Column 11: MEAN_MOTION_DDOT (double)
    if (parser.nextColumn(buf, sizeof(buf))) sat.nddot = strtod(buf, nullptr);

    OrbitData data = {
        .mean_motion = sat.n,
        .eccentricity = sat.ecc,
        .inclination = sat.incDeg,
        .raan = sat.raanDeg,
        .arg_pericenter = sat.argpDeg,
        .mean_anomaly = sat.maDeg,
        .bstar = sat.bstar,
        .mean_dot = sat.ndot,
        .mean_ddot = sat.nddot,
        .epoch = epochJd,
        .epoch_year = year,
        /*
        .epoch_month = month,
        .epoch_day = day,
        .epoch_hour = hour,
        .epoch_minute = minute,
        .epoch_second = second,*/
    };
    return data;
}

AzEl elset_to_azel(ElsetRec satrec, double jdTarget, double original_epoch, const predict_observer_t* observer) {
    // Serial1.println(satrec.error);
    double position_vector[3] = {0.0, 0.0, 0.0};
    double velocity_vector[3] = {0.0, 0.0, 0.0};

    double julianDelta = jdTarget - original_epoch;
    double minutesSinceEpoch = julianDelta * 1440.0; // 1440 minutes in a day

    sgp4(&satrec, minutesSinceEpoch, position_vector, velocity_vector);

    auto targetUnixTime = (time_t)((jdTarget - 2440587.5) * 86400.0);

    predict_observation observation = {};
    predict_position position = {
        .time     = predict_to_julian(targetUnixTime),
        .position = { position_vector[0], position_vector[1], position_vector[2] },
        .velocity = { velocity_vector[0], velocity_vector[1], velocity_vector[2] }
    };
    predict_observe_orbit(observer, &position, &observation);
    AzEl azel = {
        .azimuth = observation.azimuth * RAD_TO_DEG,
        .elevation = observation.elevation * RAD_TO_DEG
    };
    return azel;
}

auto observer = predict_create_observer(
        "home",
        my_latitude * DEG_TO_RAD,
        my_longitude * DEG_TO_RAD,
        my_altitude
    );
// OBJECT_NAME,OBJECT_ID,EPOCH,MEAN_MOTION,ECCENTRICITY,INCLINATION,RA_OF_ASC_NODE,ARG_OF_PERICENTER,MEAN_ANOMALY,EPHEMERIS_TYPE,CLASSIFICATION_TYPE,NORAD_CAT_ID,ELEMENT_SET_NO,REV_AT_EPOCH,BSTAR,MEAN_MOTION_DOT,MEAN_MOTION_DDOT
void calculateExampleSatellites() {
    while (!hasTime) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    constexpr size_t num_rows = std::size(EXAMPLE_TLES);
    for (int i = 0; i < num_rows; i+= 1) {
        auto orbit = parse_csv_gp(EXAMPLE_TLES[i]);
        example_satellites[i] = orbit;
    }
    for (int i = 0; i < num_rows; i+= 1) {
        auto orbit = example_satellites[i];
        ElsetRec satrec = {};
        satrec.whichconst = 2;
        satrec.no_kozai = orbit.mean_motion * REVS_DAY_TO_RAD_MIN;
        satrec.ecco = orbit.eccentricity;
        satrec.inclo = orbit.inclination * DEG_TO_RAD;
        satrec.nodeo = orbit.raan * DEG_TO_RAD;
        satrec.argpo = orbit.arg_pericenter * DEG_TO_RAD;
        satrec.mo = orbit.mean_anomaly * DEG_TO_RAD;
        satrec.bstar = orbit.bstar;
        satrec.ndot = orbit.mean_dot;
        satrec.nddot = orbit.mean_ddot;
        satrec.jdsatepoch = orbit.epoch;

        double jdJan1 = csvEpochToJulianDate(orbit.epoch_year, 1, 1, 0, 0, 0.0);
        satrec.epochyr = orbit.epoch_year % 100;
        satrec.epochdays = orbit.epoch - jdJan1 + 1.0;

        double jdNow = getCurrentJulianDate();

        bool success = sgp4init('i', &satrec);
        if (!success) {
            Serial1.println("SGP4 initialization failed!");
        }
        else {
            ElsetRec temp_satrec = satrec;
            AzEl azel = elset_to_azel(temp_satrec, jdNow, orbit.epoch, observer);
            if (azel.elevation > 0) {
                ScreenXY screenxy = azel_to_xy(azel);
                drawIcon(screenxy.x, screenxy.y, 0xff00ff, HUD_CIRCLE);
                AzEl previousAzEl = {
                    .azimuth = azel.azimuth,
                    .elevation = azel.elevation
                };
                constexpr double range = 1.0; // 1 day
                constexpr double step = 1.0 / 24.0 / 60.0 * 10; // 10 minutes
                for (double j = 0; j < range; j += step) {
                    temp_satrec = satrec;
                    AzEl nextAzEl = elset_to_azel(temp_satrec, jdNow + j, orbit.epoch, observer);
                    if (isnan(nextAzEl.azimuth) || isnan(nextAzEl.elevation)) {
                        continue;
                    }
                    if (nextAzEl.elevation > 0 && previousAzEl.elevation > 0) {
                        ScreenXY screenxy_prev = azel_to_xy(previousAzEl);
                        ScreenXY screenxy_next = azel_to_xy(nextAzEl);
                        canvas.drawLine(
                            (int32_t) round(screenxy_prev.x),
                            (int32_t) round(screenxy_prev.y),
                            (int32_t) round(screenxy_next.x),
                            (int32_t) round(screenxy_next.y),
                            HEX565(0x0000ff));
                    }
                    previousAzEl.azimuth = nextAzEl.azimuth;
                    previousAzEl.elevation = nextAzEl.elevation;
                }
            }
        }
    }
}

TaskHandle_t xRenderTaskHandle;
[[noreturn]]
void renderTask(void *pvParameters) {
    led_on();
    while (true) {
        drawGrid();

        calculateExampleSatellites();

        /*
        // calibrate the Az-El conversion engine
        drawIconAzEl(0, 0, 0x0275ff, HUD_CROSSHAIR_SIMPLE);
        drawIconAzEl(0, 45, 0x0275ff, HUD_CROSSHAIR_ARROWS);
        drawIconAzEl(45, 45, 0xff9100, HUD_TARGET_BOX);
        drawIconAzEl(0, 90, 0xff9100, HUD_CROSSHAIR_ARROWS);
        */

        canvas.pushSprite(0, 0);
        takeScreenshot();
        led_off();
        vTaskDelay(pdMS_TO_TICKS(1000 * 10));
        led_on();
    }
}

void calculate() {

}

TaskHandle_t xCalculateTaskHandle1;
[[noreturn]]
void calculateTask1(void *pvParameters) {
    while (true) {
        calculate();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TaskHandle_t xCalculateTaskHandle2;
[[noreturn]]
void calculateTask2(void *pvParameters) {
    while (true) {
        calculate();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

const char* getWiFiStatusName(uint8_t status) {
    switch (status) {
        case 255: return "WL_NO_SHIELD";
        case 0:   return "WL_IDLE_STATUS";
        case 1:   return "WL_NO_SSID_AVAIL";
        case 2:   return "WL_SCAN_COMPLETED";
        case 3:   return "WL_CONNECTED";
        case 4:   return "WL_CONNECT_FAILED";
        case 5:   return "WL_CONNECTION_LOST";
        case 6:   return "WL_DISCONNECTED";
        default:  return "UNKNOWN_STATUS";
    }
}

TaskHandle_t xWifiTaskHandle;
[[noreturn]]
void wifiTask(void *pvParameters) {
    led_on();
    Serial1.println("Trying to connect to Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial1.println("Wi-Fi connection request sent.");
    bool connectedOnce = false;
    while (true) {
        int wifi_status = WiFi.status();
        if (wifi_status != WL_CONNECTED) {
            digitalWrite(LED_BUILTIN, HIGH);
            Serial1.printf("WiFi status: %d (", wifi_status);
            Serial1.print(getWiFiStatusName(wifi_status));
            Serial1.println(")");
            if (connectedOnce) {
                if (WiFi.status() == WL_IDLE_STATUS || WiFi.status() == WL_DISCONNECTED || WiFi.status() == WL_CONNECT_FAILED) {
                    Serial1.println("Resetting Wi-Fi to solve disconnection");
                    WiFi.disconnect(true);
                    led_off();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    led_on();
                    WiFi.mode(WIFI_STA);
                    WiFi.begin(ssid, password);
                    led_off();
                    vTaskDelay(pdMS_TO_TICKS(30000));
                    led_on();
                }
            }
            led_off();
            vTaskDelay(pdMS_TO_TICKS(1000));
            led_on();
        } else {
            if (!connectedOnce) {
                Serial1.printf("WiFi status: %d (", wifi_status);
                Serial1.print(getWiFiStatusName(wifi_status));
                Serial1.println(")");
                xTaskNotifyGive(xTimeSyncTaskHandle);
            }
            led_off();
            vTaskDelay(pdMS_TO_TICKS(10000));
            led_on();
            connectedOnce = true;
        }
    }
}

TaskHandle_t xMemoryDiagnosticsTaskHandle;
[[noreturn]]
void memoryDiagnosticsTask(void *pvParameters) {
    led_on();
    while (true) {
        led_on();

        printMemoryStatus("diagnostics");

        led_off();
        vTaskDelay(pdMS_TO_TICKS(1000 * 60));
    }
}

void initDisplay() {
    tft.init();
    tft.setRotation(0);
    tft.initDMA();

    canvas.createSprite(240, 240);
}

void setup() {
    init_led();
    led_on();
    Serial.begin(1000000);
    Serial1.begin(115200);

    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    initDisplay();

    xTaskCreate(renderTask, "render", 2048, NULL, 2, &xRenderTaskHandle);
        printMemoryStatus("diagnostics");
    xTaskCreate(wifiTask, "wifi", 1024, NULL, 1, &xWifiTaskHandle);
        printMemoryStatus("diagnostics");
    xTaskCreate(timeSyncTask, "timesync", 1024, NULL, 1, &xTimeSyncTaskHandle);
        printMemoryStatus("diagnostics");
    xTaskCreate(calculateTask1, "calculate1", 512, NULL, 0, &xCalculateTaskHandle1);
        printMemoryStatus("calculate1");
    xTaskCreate(calculateTask2, "calculate2", 512, NULL, 0, &xCalculateTaskHandle2);
        printMemoryStatus("calculate2");
    xTaskCreate(memoryDiagnosticsTask, "memory_diagnostics", 512, NULL, 0, &xMemoryDiagnosticsTaskHandle);

    vTaskCoreAffinitySet(xCalculateTaskHandle1, (1 << 0));
    vTaskCoreAffinitySet(xCalculateTaskHandle2, (1 << 1));

    vTaskCoreAffinitySet(xWifiTaskHandle, (1 << 0));
    vTaskCoreAffinitySet(xTimeSyncTaskHandle, (1 << 0));

    vTaskCoreAffinitySet(xRenderTaskHandle, (1 << 1));
    led_off();
}

void loop() {
    vTaskDelete(NULL);
}