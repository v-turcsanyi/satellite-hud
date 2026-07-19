#include "credentials.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <TFT_eSPI.h>

#include <WiFi.h>
#include <time.h>

#include <flatbuffers/flatbuffers.h>
#include <LittleFS.h>
#include "tle_clutter_generated.h"
#include "tle_named_generated.h"

#include "../lib/sgp4/TLE.h"
#include "../lib/sgp4/SGP4.h"
#include "../lib/libpredict-2.0.0/src/unsorted.h"
#include "../lib/libpredict-2.0.0/include/predict/predict.h"

#define TLE_NAME_MAX_LEN 25
typedef struct SatelliteRenderable {
    char name[TLE_NAME_MAX_LEN];
    uint32_t norad_id;
    bool visible;
    uint8_t current_x;
    uint8_t current_y;
    uint8_t xCoordinatesPass1[128];
    uint8_t yCoordinatesPass1[128];
    uint8_t xCoordinatesPass2[128];
    uint8_t yCoordinatesPass2[128];
} SatelliteRenderable;

uint32_t norad_ids[] = {
    // various
    25544, // ISS (ZARYA)
    // L bamd
    57166, // METEOR-M2 3
    59051, // METEOR-M2 4
    38771, // METOP-B
    43689, // METOP-C
    60543, // ARCTIC WEATHER SATELLITE
    41105, // ELEKTRO-L 2
    // S band
    26958, // PROBA-1
    36037, // PROBA-2
    39159, // PROBA-V
    41240, // JASON-3
    24479, // HINODE (SOLAR-B)
    /*
    // MERIDIAN (for calibrating the algorithm as they are always visible from here)
    40296,
    44453,
    45254,
    52145,
    68571*/
};

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

#define clutter_filename "/clutter.fb"
#define named_filename "/named.fb"

SemaphoreHandle_t clutter_mutex = NULL;
SemaphoreHandle_t named_mutex = NULL;

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
    HUD_PIXEL,
};

struct OrbitData {
    double mean_motion;
    double eccentricity;
    double inclination;
    double raan;
    double arg_pericenter;
    double mean_anomaly;
    double bstar;
    // double mean_dot; // not used in SGP4
    // double mean_ddot;
    double epoch;
    /* // these fields take up space unnecessarily
    int epoch_year;
    int epoch_month;
    int epoch_day;
    int epoch_hour;
    int epoch_minute;
    double epoch_second;
    */
};

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
void drawIcon(uint8_t x, uint8_t y, uint32_t color, uint8_t radius, HudStyle style) {
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
        case HUD_PIXEL:
            canvas.drawPixel(x, y, HEX565(color));
            break;
    }
}

void takeScreenshot() {
    while (!Serial) {
        led_off();
        vTaskDelay(pdMS_TO_TICKS(100));
        led_on();
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

    this_xy.x = round((sin_azimuth * (90 - min(azel.elevation, 90)) * (120.0 / 90.0)) + 120);
    this_xy.y = round((-cos_azimuth * (90 - min(azel.elevation, 90)) * (120.0 / 90.0)) + 120);

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
    led_on();
    while (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
        led_off();
        vTaskDelay(pdMS_TO_TICKS(100));
        led_on();
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
        led_off();
        vTaskDelay(pdMS_TO_TICKS(10 * 60 * 1000));
        led_on();
    }
}

void drawIconAzEl(double az, double el, uint32_t color, uint8_t radius, HudStyle style) {
    AzEl azel = {
        .azimuth = az,
        .elevation = el
    };
    ScreenXY screenxy = azel_to_xy(azel);
    drawIcon(screenxy.x, screenxy.y, color, radius, style);
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

void parse_csv_gp(const char* csv, File output_file, const char *vip_file_path) {
    CsvRowParser parser(csv);
    flatbuffers::FlatBufferBuilder builder(512);
    char buf[32];
    TLE sat;
    char satName[25] = {0};
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
    if (strlen(epochStr) >= 19) {
        year   = atoi(epochStr);
        month  = atoi(epochStr + 5);
        day    = atoi(epochStr + 8);
        hour   = atoi(epochStr + 11);
        minute = atoi(epochStr + 14);
        second = strtod(epochStr + 17, nullptr);

        epochJd = csvEpochToJulianDate(year, month, day, hour, minute, second);
    } else {
        Serial1.print("Failed to parse ISO epoch for ");
        Serial1.print(satName);
        Serial1.print(": ");
        Serial1.println(epochStr);
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

    uint32_t norad_cat_id = 0;
    // Column 11: NORAD_CAT_ID (uint32)
    if (parser.nextColumn(buf, sizeof(buf))) norad_cat_id = strtoul(buf, nullptr, 10);

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

    bool in_vip_list = false;
    for (uint32_t norad_id: norad_ids) {
        if (norad_id == norad_cat_id) {
            in_vip_list = true;
            break;
        }
    }
    if (in_vip_list) {
        Serial1.print("\033[32m");
    }else {
        Serial1.print("\033[31m");
    }
    Serial1.print(satName);
    Serial1.println("\033[0m");
    if (in_vip_list) {
        if (xSemaphoreTake(named_mutex, portMAX_DELAY) == pdTRUE) {
            File file_vip = LittleFS.open(vip_file_path, "a");
            if (!file_vip) {
                Serial1.println("Failed to open file (named satellites) for writing");
                xSemaphoreGive(named_mutex);
                return;
            }
            const tle::OrbitParameters_named orbit_parameters{
                epochJd,
                sat.n,
                sat.ecc,
                sat.incDeg,
                sat.raanDeg,
                sat.argpDeg,
                sat.maDeg,
                sat.bstar
            };
            const auto serialized_name = builder.CreateString(satName);
            flatbuffers::Offset<tle::satellite_named> serialized = tle::Createsatellite_named(
                builder,
                serialized_name,
                norad_cat_id,
                &orbit_parameters
            );
            builder.FinishSizePrefixed(serialized);
            file_vip.write(builder.GetBufferPointer(), builder.GetSize());
            builder.Clear();
            file_vip.close();
            xSemaphoreGive(named_mutex);
        }
    }else {
        const tle::OrbitParameters_clutter orbit_parameters{
            epochJd,
            sat.n,
            sat.ecc,
            sat.incDeg,
            sat.raanDeg,
            sat.argpDeg,
            sat.maDeg,
            sat.bstar
        };
        flatbuffers::Offset<tle::satellite_clutter> serialized = tle::Createsatellite_clutter(
            builder,
            &orbit_parameters
        );
        builder.FinishSizePrefixed(serialized);
        output_file.write(builder.GetBufferPointer(), builder.GetSize());
        builder.Clear();
    }
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

void drawSun() {
    predict_observation observation = {};
    predict_julian_date_t julian_time = predict_to_julian(time(nullptr));
    predict_observe_sun(observer, julian_time, &observation);
    AzEl azel = {
        .azimuth = observation.azimuth * RAD_TO_DEG,
        .elevation = observation.elevation * RAD_TO_DEG
    };
    if (azel.elevation >= 0) {
        drawIconAzEl(azel.azimuth, azel.elevation, 0xaaaa00, 5, HUD_CIRCLE);
    }
    predict_observe_moon(observer, julian_time, &observation);
    azel = {
        .azimuth = observation.azimuth * RAD_TO_DEG,
        .elevation = observation.elevation * RAD_TO_DEG
    };
    if (azel.elevation >= 0) {
        drawIconAzEl(azel.azimuth, azel.elevation, 0x888888, 5, HUD_CIRCLE);
    }
}

void drawVipSatellites() {
    if (!hasTime) {
        return;
    }
    uint32_t lastMillis = millis();

    if (xSemaphoreTake(named_mutex, portMAX_DELAY) == pdTRUE) {
        Serial1.println("Opening the file (named) for reading");
        File file = LittleFS.open(named_filename, "r");
        if (!file) {
            Serial1.println("Failed to open file for reading");
            xSemaphoreGive(named_mutex);
            return;
        }

        Serial1.println("Starting rendering");
        size_t charIndex = 0;
        uint8_t buffer[256];
        Serial1.print("Available bytes: ");
        Serial1.println(file.available());
        Serial1.print("Size: ");
        Serial1.println(file.size());
        while (file.available() >= 4) {
            uint32_t messageSize = 0;
            file.read((uint8_t*)&messageSize, sizeof(messageSize));
            if (messageSize > sizeof(buffer)) {
                Serial1.printf("Error: message size %d exceeds local buffer!\n", messageSize);
                break;
            }
            size_t bytesRead = file.read(buffer, messageSize);
            if (bytesRead != messageSize) {
                Serial1.println("Error: incomplete read from flash.");
                break;
            }

            auto satellite = tle::Getsatellite_named(buffer);

            auto orbit = OrbitData{
                satellite->orbit()->MEAN_MOTION(),
                satellite->orbit()->ECCENTRICITY(),
                satellite->orbit()->INCLINATION(),
                satellite->orbit()->RA_OF_ASC_NODE(),
                satellite->orbit()->ARG_OF_PERICENTER(),
                satellite->orbit()->MEAN_ANOMALY(),
                satellite->orbit()->BSTAR(),
                satellite->orbit()->EPOCH(),
            };
            ElsetRec satrec = {};
            satrec.whichconst = 2;
            satrec.no_kozai = orbit.mean_motion * REVS_DAY_TO_RAD_MIN;
            satrec.ecco = orbit.eccentricity;
            satrec.inclo = orbit.inclination * DEG_TO_RAD;
            satrec.nodeo = orbit.raan * DEG_TO_RAD;
            satrec.argpo = orbit.arg_pericenter * DEG_TO_RAD;
            satrec.mo = orbit.mean_anomaly * DEG_TO_RAD;
            satrec.bstar = orbit.bstar;
            satrec.jdsatepoch = orbit.epoch;

            double jdNow = getCurrentJulianDate();

            bool success = sgp4init('i', &satrec);
            if (!success) {
                Serial1.println("SGP4 initialization failed!");
            } else {
                ElsetRec temp_satrec = satrec;
                AzEl azel = elset_to_azel(temp_satrec, jdNow, orbit.epoch, observer);
                ScreenXY screenxy = azel_to_xy(azel);
                AzEl previousAzEl = {
                    .azimuth = azel.azimuth,
                    .elevation = azel.elevation
                };
                constexpr double range = 1.0; // 1 day
                constexpr double step = 1.0 / 24.0 / 60.0; // 1 minute
                bool next_pass_found = false;
                for (double j = 0; j < range; j += step) {
                    temp_satrec = satrec;
                    AzEl nextAzEl = elset_to_azel(temp_satrec, jdNow + j, orbit.epoch, observer);
                    if (isnan(nextAzEl.azimuth) || isnan(nextAzEl.elevation)) {
                        continue;
                    }
                    if (nextAzEl.elevation > 0) {
                        next_pass_found = true;
                    }
                    if (next_pass_found) {
                        ScreenXY screenxy_prev = azel_to_xy(previousAzEl);
                        ScreenXY screenxy_next = azel_to_xy(nextAzEl);
                        canvas.drawLine(
                            (int32_t) round(screenxy_prev.x),
                            (int32_t) round(screenxy_prev.y),
                            (int32_t) round(screenxy_next.x),
                            (int32_t) round(screenxy_next.y),
                            HEX565(0x011d3f));
                        if (nextAzEl.elevation < 0) {
                            break;
                        }
                    }
                    previousAzEl.azimuth = nextAzEl.azimuth;
                    previousAzEl.elevation = nextAzEl.elevation;
                }
                if (azel.elevation > 0) {
                    for (double j = 0; j > -range; j -= step) {
                        temp_satrec = satrec;
                        AzEl nextAzEl = elset_to_azel(temp_satrec, jdNow + j, orbit.epoch, observer);
                        if (isnan(nextAzEl.azimuth) || isnan(nextAzEl.elevation)) {
                            continue;
                        }
                        ScreenXY screenxy_prev = azel_to_xy(previousAzEl);
                        ScreenXY screenxy_next = azel_to_xy(nextAzEl);
                        canvas.drawLine(
                            (int32_t) round(screenxy_prev.x),
                            (int32_t) round(screenxy_prev.y),
                            (int32_t) round(screenxy_next.x),
                            (int32_t) round(screenxy_next.y),
                            HEX565(0x011d3f));
                        if (nextAzEl.elevation < 0) {
                            break;
                        }
                        previousAzEl.azimuth = nextAzEl.azimuth;
                        previousAzEl.elevation = nextAzEl.elevation;
                    }
                    drawIcon(round(screenxy.x), round(screenxy.y), 0x0275ff, ICON_RADIUS, HUD_CROSSHAIR_ARROWS_H);
                }
            }
        }
        Serial1.print(millis() - lastMillis);
        Serial1.println(" ms");
        Serial1.println("Rendering done");
        file.close();
        xSemaphoreGive(named_mutex);
    }
}

void drawBackgroundSatellites() {
    if (!hasTime) {
        return;
    }
    uint32_t lastMillis = millis();

    if (xSemaphoreTake(clutter_mutex, 0) == pdTRUE) {
        Serial1.println("Opening the file (clutter) for reading");
        File file = LittleFS.open(clutter_filename, "r");
        if (!file) {
            Serial1.println("Failed to open file (clutter) for reading");
        }

        Serial1.println("Starting rendering");
        size_t charIndex = 0;
        uint8_t buffer[256];
        while (file.available() >= 4) {
            uint32_t messageSize = 0;
            file.read((uint8_t*)&messageSize, sizeof(messageSize));
            if (messageSize > sizeof(buffer)) {
                Serial1.printf("Error: message size %d exceeds local buffer!\n", messageSize);
                break;
            }
            size_t bytesRead = file.read(buffer, messageSize);
            if (bytesRead != messageSize) {
                Serial1.println("Error: incomplete read from flash.");
                break;
            }

            auto satellite = tle::Getsatellite_clutter(buffer);

            auto orbit = OrbitData{
                satellite->orbit()->MEAN_MOTION(),
                satellite->orbit()->ECCENTRICITY(),
                satellite->orbit()->INCLINATION(),
                satellite->orbit()->RA_OF_ASC_NODE(),
                satellite->orbit()->ARG_OF_PERICENTER(),
                satellite->orbit()->MEAN_ANOMALY(),
                satellite->orbit()->BSTAR(),
                satellite->orbit()->EPOCH(),
            };
            ElsetRec satrec = {};
            satrec.whichconst = 2;
            satrec.no_kozai = orbit.mean_motion * REVS_DAY_TO_RAD_MIN;
            satrec.ecco = orbit.eccentricity;
            satrec.inclo = orbit.inclination * DEG_TO_RAD;
            satrec.nodeo = orbit.raan * DEG_TO_RAD;
            satrec.argpo = orbit.arg_pericenter * DEG_TO_RAD;
            satrec.mo = orbit.mean_anomaly * DEG_TO_RAD;
            satrec.bstar = orbit.bstar;
            satrec.jdsatepoch = orbit.epoch;

            double jdNow = getCurrentJulianDate();

            bool success = sgp4init('i', &satrec);
            if (!success) {
                Serial1.println("SGP4 initialization failed!");
            } else {
                ElsetRec temp_satrec = satrec;
                AzEl azel = elset_to_azel(temp_satrec, jdNow, orbit.epoch, observer);
                if (azel.elevation > 0) {
                    ScreenXY screenxy = azel_to_xy(azel);
                    drawIcon(screenxy.x, screenxy.y, 0x888888, ICON_RADIUS, HUD_PIXEL);
                }
            }
        }
        Serial1.print(millis() - lastMillis);
        Serial1.println(" ms");
        Serial1.println("Rendering done");
        file.close();
        xSemaphoreGive(clutter_mutex);
    }
}

TaskHandle_t xRenderTaskHandle;
[[noreturn]]
void renderTask(void *pvParameters) {
    while (true) {
        led_on();

        drawGrid();
        drawSun();

        drawBackgroundSatellites();
        drawVipSatellites();

        /*
        // calibrate the Az-El conversion engine
        drawIconAzEl(0, 0, 0x0275ff, HUD_CROSSHAIR_SIMPLE);
        drawIconAzEl(0, 45, 0x0275ff, HUD_CROSSHAIR_ARROWS);
        drawIconAzEl(45, 45, 0xff9100, HUD_TARGET_BOX);
        drawIconAzEl(0, 90, 0xff9100, HUD_CROSSHAIR_ARROWS);
        */

        canvas.pushSprite(0, 0);
        // takeScreenshot();
        led_off();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TaskHandle_t xBgWorkerTaskHandle1;
TaskHandle_t xBgWorkerTaskHandle2;
[[noreturn]]
void bgWorkerTask(void *pvParameters) {
    Serial1.printf("Worker started on core %d\n", RP2040::cpuid());
    while (true) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {

        }
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
    while (true) {
        led_on();

        printMemoryStatus("diagnostics");

        led_off();
        vTaskDelay(pdMS_TO_TICKS(1000 * 60));
    }
}

TaskHandle_t xDownloadTaskHandle;
void downloadTask(void *pvParameters) {
    led_on();

    while (WiFi.status() != 3 /*WL_CONNECTED*/) {
        led_off();
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_on();
    }
    while (!hasTime) {
        led_off();
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_on();
    }
    uint32_t lastMillis = millis();
    if (xSemaphoreTake(named_mutex, portMAX_DELAY) == pdTRUE) {
        LittleFS.remove(named_filename);
        xSemaphoreGive(named_mutex);
    }
    if (xSemaphoreTake(clutter_mutex, portMAX_DELAY) == pdTRUE) {
        LittleFS.remove(named_filename);
        File file_clutter = LittleFS.open(clutter_filename, "w");
        if (!file_clutter) {
            Serial1.println("Failed to open file (clutter) for writing");
            file_clutter.close();
            xSemaphoreGive(clutter_mutex);
            return;
        }

        WiFiClient client;
        const char *server_ip = HTTP_SERVER_ADDRESS;
        uint16_t server_port = HTTP_PORT;
        const char *csv_path = HTTP_FILE;

        Serial1.printf("Connecting to %s:%d\n", server_ip, server_port);

        if (!client.connect(server_ip, server_port)) {
            Serial1.println("HTTP connection failed.");
            file_clutter.close();
            xSemaphoreGive(clutter_mutex);
            return;
        }
        client.printf("GET /%s HTTP/1.1\r\n", csv_path);
        client.printf("Host: %s\r\n", server_ip);
        client.print("Connection: close\r\n\r\n");

        uint8_t consecutiveNewlines = 0;
        while (client.connected() || client.available()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\r') {
                    continue; // Ignore carriage returns completely
                }
                if (c == '\n') {
                    consecutiveNewlines++;
                    if (consecutiveNewlines == 2) {
                        break; // CSV begins next byte
                    }
                } else {
                    consecutiveNewlines = 0; // Reset if we see normal characters
                }
            } else {
                led_off();
                vTaskDelay(pdMS_TO_TICKS(1));
                led_on();
            }
        }

        Serial1.println("Starting download");
        char lineBuffer[256];
        size_t charIndex = 0;

        while (client.connected() || client.available()) {
            while (client.available()) {
                char c = client.read();

                if (c == '\n') {
                    lineBuffer[charIndex] = '\0'; // Null-terminate

                    // Handle Windows-style lines (\r\n) cleanly
                    if (charIndex > 0 && lineBuffer[charIndex - 1] == '\r') {
                        lineBuffer[charIndex - 1] = '\0';
                    }

                    uint8_t comment_progress = 0;
                    if (charIndex > 0 && lineBuffer[0] != '#') {
                        if (strncmp(lineBuffer, "OBJECT_NAME", 11) == 0) {
                            // Serial1.println("Comment line skipped");
                            charIndex = 0;
                            continue;
                        }
                        // Serial1.println(lineBuffer);

                        // list of operators that pollute the list with temporary satellites
                        // drop them to save space, expand this list in case
                        // more operators start launching temporary satellites
                        if (strstr(lineBuffer, "STARLINK") != NULL) {
                            charIndex = 0;
                            continue;
                        }

                        parse_csv_gp(lineBuffer, file_clutter, named_filename);
                    }

                    charIndex = 0; // Reset index for next line
                } else {
                    if (charIndex < sizeof(lineBuffer) - 1) {
                        lineBuffer[charIndex++] = c;
                    }
                }
            }
            // Yield for 1 tick to let the lwIP Wi-Fi stack process background packets
            led_off();
            vTaskDelay(pdMS_TO_TICKS(1));
            led_on();
        }
        client.stop();
        file_clutter.close();
        xSemaphoreGive(clutter_mutex);
        Serial1.print(millis() - lastMillis);
        Serial1.println(" ms");
        Serial1.println("Download completed");
    }

    led_off();
    vTaskDelete(NULL);
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
    Serial1.begin(2000000);

    if (!LittleFS.begin()) {
        Serial1.println("An error has occurred while mounting LittleFS!");
        vTaskDelete(NULL);
    }
    Serial1.println("LittleFS Mounted Successfully!");

    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    initDisplay();

    clutter_mutex = xSemaphoreCreateMutex();
    if (clutter_mutex == NULL) {
        Serial1.println("Failed to create flash mutex (clutter)");
    }
    named_mutex = xSemaphoreCreateMutex();
    if (named_mutex == NULL) {
        Serial1.println("Failed to create flash mutex (named)");
    }

    xTaskCreate(downloadTask, "download", 2048, NULL, 0, &xDownloadTaskHandle);
    printMemoryStatus("download");
    xTaskCreate(renderTask, "render", 2048, NULL, 2, &xRenderTaskHandle);
    printMemoryStatus("diagnostics");
    xTaskCreate(wifiTask, "wifi", 1024, NULL, 1, &xWifiTaskHandle);
    printMemoryStatus("diagnostics");
    xTaskCreate(timeSyncTask, "timesync", 1024, NULL, 1, &xTimeSyncTaskHandle);
    printMemoryStatus("diagnostics");
    xTaskCreate(bgWorkerTask, "calculate1", 512, NULL, 0, &xBgWorkerTaskHandle1);
    printMemoryStatus("calculate1");
    xTaskCreate(bgWorkerTask, "calculate2", 512, NULL, 0, &xBgWorkerTaskHandle2);
    printMemoryStatus("calculate2");
    xTaskCreate(memoryDiagnosticsTask, "memory_diagnostics", 512, NULL, 3, &xMemoryDiagnosticsTaskHandle);

    vTaskCoreAffinitySet(xBgWorkerTaskHandle1, (1 << 0));
    vTaskCoreAffinitySet(xBgWorkerTaskHandle2, (1 << 1));

    vTaskCoreAffinitySet(xWifiTaskHandle, (1 << 0));
    vTaskCoreAffinitySet(xTimeSyncTaskHandle, (1 << 0));

    vTaskCoreAffinitySet(xRenderTaskHandle, (1 << 1));
    led_off();
}

void loop() {
    vTaskDelete(NULL);
}