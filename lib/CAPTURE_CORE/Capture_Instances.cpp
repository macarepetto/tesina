#include "Capture_Instances.h"

namespace CaptureInstances {
PcntExtended& pcnt() {
    static PcntExtended counter;
    return counter;
}
PeriodicTickCapture& pps() {
    static PeriodicTickCapture capture(pcnt());
    return capture;
}
PeriodicTickCapture& sqw() {
    static PeriodicTickCapture capture(pcnt());
    return capture;
}
ButtonTickCapture& button() {
    static ButtonTickCapture capture(pcnt(), pps(), sqw());
    return capture;
}
}
