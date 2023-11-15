
#include <stdio.h>
#include <rc/adc.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include "spdlog/spdlog.h"

#include "robot_app.h"
#include "robot_battery.h"

#define VOLTAGE_DISCONNECT 1 // Threshold for detecting disconnected battery

BatteryStatus batteryStatus;

void *batteryStatusUpdater(void *unused)
{
	robotStatus.batteryStatus = &batteryStatus;
	double pack_voltage; // 2S pack voltage on JST XH 2S balance connector
	double cell_voltage; // cell voltage
	double jack_voltage; // could be dc power supply or another battery

	if (rc_adc_init() == -1)
	{
		robotStatus.batteryStatus->error = 1;
		return nullptr;
	}

	while (rc_get_state() != EXITING)
	{
		// read in the voltage of the 2S pack and DC jack
		pack_voltage = rc_adc_batt();
		jack_voltage = rc_adc_dc_jack();

		// sanity check the SDC didn't return an error
		if (pack_voltage < 0.0 || jack_voltage < 0.0)
		{
			spdlog::error("Can't read voltages");
			batteryStatus.error = true;
		}

		if (batteryStatus.error == false)
		{
			// check if a pack is on the 2S balance connector
			if (pack_voltage < VOLTAGE_DISCONNECT)
			{
				pack_voltage = 0;
			}

			if (jack_voltage < VOLTAGE_DISCONNECT)
			{
				jack_voltage = 0;
			}

			// 2S pack, so divide by two for cell voltage
			cell_voltage = pack_voltage / 2;

			batteryStatus.cell_voltage = cell_voltage;
			batteryStatus.jack_voltage = jack_voltage;
			batteryStatus.pack_voltage = pack_voltage;
		}
		// check periodically
		rc_usleep(100000);
	}

	rc_adc_cleanup();
}
