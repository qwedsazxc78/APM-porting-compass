/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_VRBRAIN.h>

#include <AP_Math.h>    // ArduPilot Mega Vector/Matrix math Library
#include <AP_Declination.h>
#include <AP_Compass.h> // Compass Library

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define CONFIG_COMPASS HAL_COMPASS_DEFAULT

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_IST8303
static AP_Compass_IST8303 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

uint32_t timer;

void setup() {
    hal.console->println("Compass library test");

    if (!compass.init()) {
        hal.console->println("compass initialisation failed!");
        while (1) ;
    }
    hal.console->println("init done");

    compass.set_and_save_offsets(0,0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

    hal.console->print("Compass auto-detected as: ");
    switch( compass.product_id ) {
    case AP_COMPASS_TYPE_HIL:
        hal.console->println("HIL");
        break;
    case AP_COMPASS_TYPE_HMC5843:
        hal.console->println("HMC5843");
        break;
    case AP_COMPASS_TYPE_HMC5883L:
        hal.console->println("HMC5883L");
        break;
    case AP_COMPASS_TYPE_IST8303:
        hal.console->println("IST8303");
        break;
    case AP_COMPASS_TYPE_PX4:
        hal.console->println("PX4");
        break;
    default:
        hal.console->println("unknown");
        break;
    }

    hal.scheduler->delay(1000);
    timer = hal.scheduler->micros();
}

void loop()
{ 

    compass.accumulate();

    if((hal.scheduler->micros()- timer) > 100000L)
    {
        timer = hal.scheduler->micros();
        compass.read();
        unsigned long read_time = hal.scheduler->micros() - timer;
        float heading;

        if (!compass.healthy()) {
            hal.console->println("not healthy");
            return;
        }
    Matrix3f dcm_matrix;
    // use roll = 0, pitch = 0 for this example
    dcm_matrix.from_euler(0, 0, 0);
        heading = compass.calculate_heading(dcm_matrix);
        compass.learn_offsets();

        // capture min
        const Vector3f &mag = compass.get_field();
        

        // display all to user
        hal.console->printf("Heading: %.2f (%3d,%3d,%3d) i2c error: %u",
                ToDeg(heading),
                (int)mag.x,
                (int)mag.y,
                (int)mag.z, 
                (unsigned)hal.i2c->lockup_count());

        // display offsets
        

        hal.console->printf(" t=%u", (unsigned)read_time);

        hal.console->println();
    } else {
        hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();

