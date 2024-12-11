/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for AUAV-L05D combined differential airspeed & absolute barometer sensor
  https://allsensors.com/en/product-listing?family=auav
 */

#include "AP_Airspeed_AUAV.h"

#if AP_AIRSPEED_AUAV_ENABLED

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
extern const AP_HAL::HAL &hal;

#ifdef AUAV_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif


AP_Airspeed_AUAV::AP_Airspeed_AUAV(AP_Airspeed &_frontend, uint8_t _instance, const float _range_inH2O) :
    AP_Airspeed_Backend(_frontend, _instance),
    range_inH2O(_range_inH2O)
{}

/*
  probe for a sensor on a given i2c address
 */
AP_Airspeed_Backend *AP_Airspeed_AUAV::probe(AP_Airspeed &_frontend,
                                             uint8_t _instance,
                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev,
                                             const float _range_inH2O)  // Todo: probably not fiding the sensor. It is tryng to read something (reg 0???) but no answer.
{
    if (!_dev) {
        return nullptr;
    }
    AP_Airspeed_AUAV *sensor = NEW_NOTHROW AP_Airspeed_AUAV(_frontend, _instance, _range_inH2O);
    if (!sensor) {
        return nullptr;
    }
    sensor->dev = std::move(_dev);
    sensor->setup();
    return sensor;
}

// initialise the sensor
void AP_Airspeed_AUAV::setup()
{
    Debug("AUAV: Started setup code");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: Started setup code");
    WITH_SEMAPHORE(dev->get_semaphore());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: setup got semaphore");
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: setup speed low");
    dev->set_retries(2);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: setup retries 2");
    dev->set_device_type(uint8_t(DevType::AUAV));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: setup devtype auav");
    set_bus_id(dev->get_bus_id());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: setup got bus id");
    // Send Start-Average16 command to start measurement
    uint8_t command[] = {START_AVERAGE2_CMD};
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: setup created start command %u", command[0]);
    if (!dev->transfer(command, 1, nullptr, 0)) { 
        Debug("AUAV: Failed to send Start-Average2 command");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: setup sent start command");
    // Register periodic callback for differential pressure sensor
    Debug("AUAV: Start periodic callback");
    dev->register_periodic_callback(1000000UL/50U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_AUAV::timer, void));
    Debug("AUAV: Started periodic callback, finished setup");
}

// probe and initialise the sensor
bool AP_Airspeed_AUAV::init()
{
    Debug("AUAV: Started init code");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: Started init code");
    dev = hal.i2c_mgr->get_device(get_bus(), AUAVDIFF_I2C_ADDR);
    if (!dev) {
        Debug("AUAV: finished init !dev");
        return false;
    }
    setup();
    Debug("AUAV: finished init code dev");
    return true;
}

// 50Hz timer
void AP_Airspeed_AUAV::timer()
{
    // Sensor has to be set to specific mode (averaging) suggestion: average16 for less noise and less computing power
    Debug("AUAV: Started timer code");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV: Started timer code");
    uint8_t raw_bytes[7];
    uint8_t status;
    uint32_t pressure_raw;
    uint32_t temperature_raw;
    if (!dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
        // return;
        status = 0x03;
        pressure_raw = (0xAA << 16) |
                                (0xAA << 8) |
                                0xAA;
        temperature_raw = (0xAA << 16) |
                                (0xAA << 8) |
                                0xAA;
    }
    else {
        // Extract status, pressure, and temperature
        status = raw_bytes[0];
        pressure_raw = (raw_bytes[1] << 16) |
                                (raw_bytes[2] << 8) |
                                raw_bytes[3];
        temperature_raw = (raw_bytes[4] << 16) |
                                    (raw_bytes[5] << 8) |
                                    raw_bytes[6];
    }

    

    // Check status byte
    if ((status & 0x03) != 0) {
        Debug("AUAV: Bad status read %u", status);
        return;
    }

    float press_h2o = 1.25f * (pressure_raw - 8388608.0f) / 16777216.0f * (2.0f * range_inH2O);


    if ((press_h2o > range_inH2O) || (press_h2o < -range_inH2O)) {
        Debug("AUAV: Out of range pressure %f", press_h2o);
        return;
    }

    float temp = temperature_raw * (155.0f / 16777216.0f) - 45.0f;

    WITH_SEMAPHORE(sem);

    const uint32_t now = AP_HAL::millis();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal" // suppress -Wfloat-equal as we are only worried about the case where we had never read
    if ((temperature != 0) && (fabsf(temperature - temp) > 10) && ((now - last_sample_time_ms) < 2000)) {
        Debug("AUAV: Temperature swing %f", temp);
        return;
    }
#pragma GCC diagnostic pop

    pressure_sum += INCH_OF_H2O_TO_PASCAL * press_h2o;
    temperature_sum += temp;
    press_count++;
    temp_count++;
    last_sample_time_ms = now;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_AUAV::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }

    _pressure = pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_AUAV::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (temp_count > 0) {
        temperature = temperature_sum / temp_count;
        temp_count = 0;
        temperature_sum = 0;
    }

    _temperature = temperature;
    return true;
}

#endif
