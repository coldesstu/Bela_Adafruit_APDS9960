#include <Bela.h>
#include "Bela_Adafruit_APDS9960.h"
#include <I2c.h>

const int INT_PIN = 1;  // adjust this as per your wiring

Adafruit_APDS9960 apds;

// Interval for reading from the sensor
int gPrintInterval = 500; //ms
int gPrintIntervalSamples = 0;
// Sleep time for auxiliary task
unsigned int gTaskSleepTime = 12000; // microseconds

void loop(void*)
{
	while(!Bela_stopRequested()) {
		apds.readProximity();
		usleep(gTaskSleepTime);
	}
}

bool setup(BelaContext *context, void *userData)
{
	// Set pin as input
    pinMode(context, 0, INT_PIN, INPUT);
    
	if(!apds.begin()){
        rt_printf("Failed to initialize device! Please check your wiring.\n");
        return false;
    }
    else rt_printf("Device initialized!\n");

    // Enable proximity mode
    apds.enableProximity(true);

    // Set the interrupt threshold to fire when proximity reading goes above 175
    apds.setProximityInterruptThreshold(0, 175);

    // Enable the proximity interrupt
    apds.enableProximityInterrupt();
    
    
    return true;
	//apds.printDetails();

	Bela_runAuxiliaryTask(loop);

	gPrintIntervalSamples = context->audioSampleRate*(gPrintInterval/1000.0);
	return true;
}

void render(BelaContext *context, void *userData)
{
	static int readCount = 0;
	for(unsigned int n = 0; n < context->audioFrames; n++) {
		if(readCount >= gPrintIntervalSamples) {
			readCount = 0;
			rt_printf("%d\n", apds.readProximity());
		}
		readCount++;
	}
}

void cleanup(BelaContext *context, void *userData)
{
}
