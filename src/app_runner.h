#ifndef APP_RUNNER_H
#define APP_RUNNER_H

#include "IMotorSystem.h"

class AppRunner {
public:
    static void start(IMotorSystem* motorSystem);
};

#endif // APP_RUNNER_H
