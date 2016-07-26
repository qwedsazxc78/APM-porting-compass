/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_Compass_HMC5843.cpp - Arduino Library for HMC5843 I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_IST8303.h"

extern const AP_HAL::HAL& hal;

#define COMPASS_ADDRESS      0x0C
#define ConfigRegA           0x0A
#define ConfigRegB           0x0B
#define MAG_WHOAMI           0x00
#define MAG_IS8303ID         0xFF
#define IST8303_AVERAGE      0x41

#define DataOutputRate_100HZ 0x06
#define IST8303_AVG_16_TIME  0x24
// #define magGain              0x20
// #define PositiveBiasConfig   0x11
// #define NegativeBiasConfig   0x12
// #define NormalOperation      0x10
// #define ModeRegister         0x02
// #define ContinuousConversion 0x00
// #define SingleConversion     0x01

// ConfigRegA valid sample averaging for IST8303
// #define SampleAveraging_1    0x00
// #define SampleAveraging_2    0x01
// #define SampleAveraging_4    0x02
// #define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for IST8303
// #define DataOutputRate_0_75HZ 0x00
// #define DataOutputRate_1_5HZ  0x01
// #define DataOutputRate_3HZ    0x02
// #define DataOutputRate_7_5HZ  0x03
// #define DataOutputRate_15HZ   0x04
// #define DataOutputRate_30HZ   0x05
// #define DataOutputRate_75HZ   0x06


// read_register - read a register value
bool AP_Compass_IST8303::read_register(uint8_t address, uint8_t *value)
{
    if (hal.i2c->readRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _healthy[0] = false;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_IST8303::write_register(uint8_t address, uint8_t value)
{
    if (hal.i2c->writeRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _healthy[0] = false;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_IST8303::read_raw()
{
    uint8_t buff[6];

    if (hal.i2c->readRegisters(COMPASS_ADDRESS, 0x03, 6, buff) != 0) {
        if (_healthy[0]) {
            hal.i2c->setHighSpeed(false);
        }
        _healthy[0] = false;
        _i2c_sem->give();
        return false;
    }

    int16_t rx, ry, rz;
    rx = (((int16_t)buff[1]) << 8) | buff[0];
    ry = (((int16_t)buff[3]) << 8) | buff[2];
    rz = (((int16_t)buff[5]) << 8) | buff[4];

    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }


    _mag_x = rx;
    _mag_y = ry;
    _mag_z = rz;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_IST8303::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
    uint32_t tnow = hal.scheduler->micros();
    if (_healthy[0] && _accum_count != 0 && (tnow - _last_accum_time) < 13333) {
        // the compass gets new data at 75Hz
        return;
    }

    if (!_i2c_sem->take(1)) {
        // the bus is busy - try again later
        return;
    }
    bool result = read_raw();
    _i2c_sem->give();

    if (result) {
        // the _mag_N values are in the range -2048 to 2047, so we can
        // accumulate up to 15 of them in an int16_t. Let's make it 14
        // for ease of calculation. We expect to do reads at 10Hz, and
        // we get new data at most 75Hz, so we don't expect to
        // accumulate more than 8 before a read
        _mag_x_accum += _mag_x;
        _mag_y_accum += _mag_y;
        _mag_z_accum += _mag_z;
        _accum_count++;
        if (_accum_count == 14) {
            _mag_x_accum /= 2;
            _mag_y_accum /= 2;
            _mag_z_accum /= 2;
            _accum_count = 7;
        }
        _last_accum_time = tnow;
    }
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_IST8303::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) )
        return false;
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool AP_Compass_IST8303::init()
{
    int numAttempts = 0, good_count = 0;
    bool success = false;
    bool initREG = false;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;

    hal.scheduler->delay(10);

    _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get IST8303 semaphore"));
    }

    // determine if we are using IST8303
    _base_config = 0;
    initREG = write_register(IST8303_AVERAGE, IST8303_AVG_16_TIME) && write_register(ConfigRegA, DataOutputRate_100HZ);

    if (!initREG || !read_register(ConfigRegA, &_base_config) ) {
        _healthy[0] = false;
        _i2c_sem->give();
        return false;
    }


    if ( _base_config == DataOutputRate_100HZ ) {
        // a 5883L supports the sample averaging config
        product_id = AP_COMPASS_TYPE_IST8303;
    } else {
        // not behaving like either supported compass type
        _i2c_sem->give();
        return false;
    }



    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    while ( success == 0 && numAttempts < 25 && good_count < 5)
    {
        // record number of attempts at initialisation
        numAttempts++;

        // read values from the compass
        hal.scheduler->delay(50);
        if (!read_raw())
            continue;      // we didn't read valid values

        hal.scheduler->delay(10);

        float cal[3];

        hal.console->printf_P(PSTR("mag %d %d %d\n"), _mag_x, _mag_y, _mag_z);
        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

        hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f\n"), cal[0], cal[1], cal[2]);

        // we throw away the first two samples as the compass may
        // still be changing its state from the application of the
        // strap excitation. After that we accept values in a
        // reasonable range
        if (numAttempts > 2 &&
                cal[0] > 0.7f && cal[0] < 1.35f &&
                cal[1] > 0.7f && cal[1] < 1.35f &&
                cal[2] > 0.7f && cal[2] < 1.35f) {
            hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f good\n"), cal[0], cal[1], cal[2]);
            good_count++;
            calibration[0] += cal[0];
            calibration[1] += cal[1];
            calibration[2] += cal[2];
        }

#if 0
        /* useful for debugging */
        hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), cal[0], cal[1], cal[2]);
#endif
    }


    if (good_count >= 5) {
        /*
          The use of gain_multiple below is incorrect, as the gain
          difference between 2.5Ga mode and 1Ga mode is already taken
          into account by the expected_x and expected_yz values.  We
          are not going to fix it however as it would mean all
          APM1/APM2 users redoing their compass calibration. The
          impact is that the values we report on APM1/APM2 are lower
          than they should be (by a multiple of about 0.6). This
          doesn't have any impact other than the learned compass
          offsets
         */
        calibration[0] = calibration[0] * gain_multiple / good_count;
        calibration[1] = calibration[1] * gain_multiple / good_count;
        calibration[2] = calibration[2] * gain_multiple / good_count;
        success = true;
    } else {
        /* best guess */
        calibration[0] = 1.0;
        calibration[1] = 1.0;
        calibration[2] = 1.0;
    }

    // leave test mode
    if (!re_initialise()) {
        _i2c_sem->give();
        return false;
    }

    _i2c_sem->give();
    _initialised = true;

    // perform an initial read
    _healthy[0] = true;
    read();
#if 0
    hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"),
                          calibration[0], calibration[1], calibration[2]);
#endif

    success = true;
    return success;
}

// Read Sensor data
bool AP_Compass_IST8303::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return false;
    }
    if (!_healthy[0]) {
        if (hal.scheduler->millis() < _retry_time) {
            return false;
        }
        if (!re_initialise()) {
            _retry_time = hal.scheduler->millis() + 1000;
            hal.i2c->setHighSpeed(false);
            return false;
        }
    }

    if (_accum_count == 0) {
        accumulate();
        if (!_healthy[0] || _accum_count == 0) {
            // try again in 1 second, and set I2c clock speed slower
            _retry_time = hal.scheduler->millis() + 1000;
            hal.i2c->setHighSpeed(false);
            return false;
        }
    }

    // add soft - iron and hard - iron calibration fot ist8308
    // 20160718 Alex
    // float hardiron_offset[3] = {0, 0, 0};
    // float softiron_cali[3][3] = {
    //     {1, 0, 0}, { 0, 1, 0}, {0, 0, 1 }
    // };
    float hardiron_offset[3] = { -33.5942, -26.6654, 15.8412};
    float softiron_cali[3][3] = {
        {0.9777, -0.00137853, 0.0227123 }, { -0.00137593, 1.02076, -0.0345169}, {0.0227065, -0.0345167, 1.0037 }
    };

    float tmpMx, tmpMy, tmpMz;
    float LSB2mG = 3;
    tmpMx = (_mag_x_accum * calibration[0] / _accum_count) - hardiron_offset[0];
    tmpMy = (_mag_y_accum * calibration[1] / _accum_count) - hardiron_offset[1];
    tmpMz = (_mag_z_accum * calibration[2] / _accum_count) - hardiron_offset[2];
    _field[0].x = softiron_cali[0][0] * tmpMx + softiron_cali[0][1] * tmpMy  + softiron_cali[0][2] * tmpMz;
    _field[0].y = softiron_cali[1][0] * tmpMx + softiron_cali[1][1] * tmpMy  + softiron_cali[1][2] * tmpMz;
    _field[0].z = softiron_cali[2][0] * tmpMx + softiron_cali[2][1] * tmpMy  + softiron_cali[2][2] * tmpMz;
    _field[0].x *= LSB2mG;
    _field[0].y *= LSB2mG;
    _field[0].z *= LSB2mG;

    // original data
    // _field[0].x = (_mag_x_accum * calibration[1] / _accum_count);
    // _field[0].y = (_mag_y_accum * calibration[1] / _accum_count);
    // _field[0].z = (_mag_z_accum * calibration[2] / _accum_count);

    _accum_count = 0;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    last_update = hal.scheduler->micros(); // record time of update

    // rotate to the desired orientation
    if (product_id == AP_COMPASS_TYPE_IST8303) {
        _field[0].rotate(ROTATION_NONE);
    }

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _field[0].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _field[0].rotate((enum Rotation)_orientation.get());

    if (!_external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _field[0].rotate(_board_orientation);
    }

    _field[0] += _offset[0].get();

    // apply motor compensation
    if (_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) {
        _motor_offset[0] = _motor_compensation[0].get() * _thr_or_curr;
        _field[0] += _motor_offset[0];
    } else {
        _motor_offset[0].zero();
    }

    _healthy[0] = true;

    return true;
}
