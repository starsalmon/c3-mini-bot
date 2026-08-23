#pragma once

class BatteryMonitor;
class DriveServos;
class PanServo;
class IrTx;

void bot_power_register(DriveServos *drive, PanServo *pan, IrTx *ir_tx);
void bot_halt_low_battery(const BatteryMonitor &battery);
