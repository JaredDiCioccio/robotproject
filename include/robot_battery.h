#ifndef BATTERY_H
#define BATTERY_H

typedef struct BatteryStatus
{
    double pack_voltage; // 2S pack voltage on JST XH 2S balance connector
    double cell_voltage; // cell voltage
    double jack_voltage; // could be dc power supply or another battery
    bool error;
} BatteryStatus;


#endif