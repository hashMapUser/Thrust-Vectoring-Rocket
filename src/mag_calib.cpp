#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include "mag_calib.h"
#include "mmc5603nj.h"

bool mag_calibrate(MagCalib *cal) {
    Serial.println("  Mag calibration — rotate sensor through a full sphere.");
    Serial.println("  Rotate on all axes for ~50 seconds.");
    Serial.println("  Starting in 3 seconds...");
    delay(3000);

    float min_x =  9999, max_x = -9999;
    float min_y =  9999, max_y = -9999;
    float min_z =  9999, max_z = -9999;

    for (int i = 0; i < MAG_CALIB_SAMPLES; i++) {
        MMC5603NJ_Data d;
        mmc5603nj_read(&d);

        if (!d.valid) {
            Serial.println("  [WARN] Bad read during calibration — skipping sample");
            continue;
        }

        if (d.mag_x < min_x) min_x = d.mag_x;
        if (d.mag_x > max_x) max_x = d.mag_x;
        if (d.mag_y < min_y) min_y = d.mag_y;
        if (d.mag_y > max_y) max_y = d.mag_y;
        if (d.mag_z < min_z) min_z = d.mag_z;
        if (d.mag_z > max_z) max_z = d.mag_z;

        if ((i + 1) % 50 == 0) {
            Serial.print("  ... "); Serial.print((i + 1) * 100 / MAG_CALIB_SAMPLES);
            Serial.println("%");
        }

        delay(100);  // ~10 Hz
    }

    // Check that we actually got spread on all axes
    float spread_x = max_x - min_x;
    float spread_y = max_y - min_y;
    float spread_z = max_z - min_z;

    Serial.print("  Spread — x="); Serial.print(spread_x, 4);
    Serial.print(" y="); Serial.print(spread_y, 4);
    Serial.print(" z="); Serial.println(spread_z, 4);

    if (spread_x < 0.05f || spread_y < 0.05f || spread_z < 0.05f) {
        Serial.println("  [FAIL] Insufficient spread on one or more axes.");
        Serial.println("  Rotate more aggressively and retry.");
        return false;
    }

    // Hard iron: midpoint of min/max per axis
    cal->offset_x = (max_x + min_x) / 2.0f;
    cal->offset_y = (max_y + min_y) / 2.0f;
    cal->offset_z = (max_z + min_z) / 2.0f;

    // Soft iron: normalize each axis range to the average range
    // so the ellipsoid becomes a sphere
    float avg_range = (spread_x + spread_y + spread_z) / 3.0f;
    cal->scale_x = avg_range / spread_x;
    cal->scale_y = avg_range / spread_y;
    cal->scale_z = avg_range / spread_z;

    Serial.println("  Calibration complete:");
    Serial.print("    Hard iron offset — x="); Serial.print(cal->offset_x, 4);
    Serial.print(" y="); Serial.print(cal->offset_y, 4);
    Serial.print(" z="); Serial.println(cal->offset_z, 4);
    Serial.print("    Soft iron scale  — x="); Serial.print(cal->scale_x, 4);
    Serial.print(" y="); Serial.print(cal->scale_y, 4);
    Serial.print(" z="); Serial.println(cal->scale_z, 4);

    return true;
}

void mag_apply_calib(const MagCalib *cal,
                     float mx,  float my,  float mz,
                     float *cx, float *cy, float *cz) {
    *cx = (mx - cal->offset_x) * cal->scale_x;
    *cy = (my - cal->offset_y) * cal->scale_y;
    *cz = (mz - cal->offset_z) * cal->scale_z;
}

void mag_save_calib(const MagCalib *cal) {
    int addr = MAG_CALIB_EEPROM_ADDR;
    uint16_t magic = MAG_CALIB_MAGIC;
    EEPROM.put(addr, magic);             addr += sizeof(uint16_t);
    EEPROM.put(addr, cal->offset_x);     addr += sizeof(float);
    EEPROM.put(addr, cal->offset_y);     addr += sizeof(float);
    EEPROM.put(addr, cal->offset_z);     addr += sizeof(float);
    EEPROM.put(addr, cal->scale_x);      addr += sizeof(float);
    EEPROM.put(addr, cal->scale_y);      addr += sizeof(float);
    EEPROM.put(addr, cal->scale_z);
    Serial.println("  Mag calibration saved to EEPROM.");
}

bool mag_load_calib(MagCalib *cal) {
    int addr = MAG_CALIB_EEPROM_ADDR;
    uint16_t magic = 0;
    EEPROM.get(addr, magic);             addr += sizeof(uint16_t);

    if (magic != MAG_CALIB_MAGIC) {
        Serial.println("  [WARN] No valid mag calibration in EEPROM — run mag_calibrate().");
        cal->offset_x = cal->offset_y = cal->offset_z = 0.0f;
        cal->scale_x  = cal->scale_y  = cal->scale_z  = 1.0f;
        return false;
    }

    EEPROM.get(addr, cal->offset_x);     addr += sizeof(float);
    EEPROM.get(addr, cal->offset_y);     addr += sizeof(float);
    EEPROM.get(addr, cal->offset_z);     addr += sizeof(float);
    EEPROM.get(addr, cal->scale_x);      addr += sizeof(float);
    EEPROM.get(addr, cal->scale_y);      addr += sizeof(float);
    EEPROM.get(addr, cal->scale_z);

    Serial.println("  Mag calibration loaded from EEPROM.");
    return true;
}