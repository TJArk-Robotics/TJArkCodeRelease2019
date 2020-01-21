#include "Platform/Nao/NaoBody.h"
#include "Platform/Thread.h"
#include "Platform/ThreadManager.h"
#include "Processes/Cognition.h"
#include "Processes/Debug.h"
#include "Processes/Motion.h"
#include "Processes/UpperCognition.h"
#include "Processes/LowerCognition.h"
#include "Processes/Behavior.h"
#include "Tools/Module/Blackboard.h"

#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <unistd.h>

bool attemptingShutdown = false;

static bool run = true;

static void sighandlerSegmentation(int sig)
{
    if (run)
    {
        printf("Caught signal %i\nShutting down...\n", sig);
    }
    // exit(EXIT_SUCCESS);
    // run = false;
    // attemptingShutdown = true;
}

static void sighandlerShutdown(int sig)
{
    if (run)
    {
        printf("Caught signal %i\nShutting down...\n", sig);
    }
    run = false;
    attemptingShutdown = true;
}

int main(int argc, char **argv)
{
    NaoBody naoBody;
    if (!naoBody.init())
    {
        std::cout << "[INFO] Waiting for NaoQi/libagent..." << std::endl;
        do
        {
            usleep(1000000);
            std::cout << "[INFO] waiting" << std::endl;
        } while (!naoBody.init());
    }

    std::cout << "[INFO] Connected to lola_connector" << std::endl;

    Thread::name = "Main";

    Blackboard *blackboard;

    blackboard = new Blackboard;

    std::cout << "[INFO] Main: blackboard: " << blackboard << std::endl;

    signal(SIGINT, sighandlerShutdown);
    signal(SIGTERM, sighandlerShutdown);
    // signal(SIGSEGV, sighandlerSegmentation);
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);

    BH_TRACE_INIT("Main");

    /**--> Open Two Camera */
    ThreadManager upper("Upper", 33000);
    upper.run<UpperCognition>(blackboard);

    while(!blackboard->camTopOpen)
    {
        usleep(1000);
    }

    ThreadManager lower("Lower", 33000);
    lower.run<LowerCognition>(blackboard);

    while(!blackboard->camLowOpen)
    {
        usleep(1000);
    }
    /* Open Two Camera <--**/

    // ThreadManager cognition("Cognition", 33000);
    // cognition.run<Cognition>(blackboard);

    ThreadManager motion("Motion", 0);
    motion.run<Motion>(blackboard);

    ThreadManager behavior("Behavior", 33000);
    behavior.run<Behavior>(blackboard);

    ThreadManager debug("Debug", 33000);
    debug.run<Debug>(blackboard);

    while (run)
    {
        pause();
    }

    delete blackboard;

    exit(EXIT_SUCCESS);
}
