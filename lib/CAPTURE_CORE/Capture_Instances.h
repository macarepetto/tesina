#pragma once
#include "Pcnt_Extended.h"
#include "Tick_Capture.h"

// Estos cuatro objetos se construyen UNA sola vez y comparten PCNT_UNIT_0.
namespace CaptureInstances {
PcntExtended& pcnt();
PeriodicTickCapture& pps();
PeriodicTickCapture& sqw();
ButtonTickCapture& button();
}
