#include "SoundPlayer.h"
#include "Platform/SystemCall.h"

#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>

SystemCall::Mode SystemCall::getMode()
{
    return physicalRobot;
}

void SystemCall::getLoad(float &mem, float load[3])
{
    struct sysinfo info;
    if (sysinfo(&info) == -1)
    {
        load[0] = load[1] = load[2] = mem = -1.f;
    }
    else
    {
        load[0] = float(info.loads[0]) / 65536.f;
        load[1] = float(info.loads[1]) / 65536.f;
        load[2] = float(info.loads[2]) / 65536.f;
        mem = float(info.totalram - info.freeram) / float(info.totalram);
    }
}

int SystemCall::playSound(const char *name)
{
    // printf("[INFO] Playing %s\n", name);
    return 0;
}

bool SystemCall::soundIsPlaying()
{
    return true;
}

bool SystemCall::usbIsMounted()
{
    return system("mount | grep \"media/usb\" >/dev/null") == 0;
}