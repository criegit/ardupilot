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
#pragma once

// backend driver for AllSensors AUAV differential airspeed sensor
// currently assumes a 5" of water, noise reduced, sensor

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_AUAV_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

// I2C Adress of differntial pressure sensor
#define AUAVDIFF_I2C_ADDR 0x38

// Commands to start measuring modes
#define START_Single_CMD 0xAA
#define START_AVERAGE2_CMD 0xAC
#define START_AVERAGE4_CMD 0xAD
#define START_AVERAGE8_CMD 0xAE
#define START_AVERAGE16_CMD 0xAF



class AP_Airspeed_AUAV : public AP_Airspeed_Backend
{
public:

    AP_Airspeed_AUAV(AP_Airspeed &frontend, uint8_t _instance, const float _range_inH2O);
    static AP_Airspeed_Backend *probe(AP_Airspeed &frontend, uint8_t _instance, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev, const float _range_inH2O);

    ~AP_Airspeed_AUAV(void) {}
    
    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void timer();

    float pressure;
    float temperature;
    float temperature_sum;
    float pressure_sum;
    uint32_t temp_count;
    uint32_t press_count;
    
    uint32_t last_sample_time_ms;
    const float range_inH2O;

    // initialise the sensor
    void setup();
    
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
};

#endif  // AP_AIRSPEED_AUAV_ENABLED
