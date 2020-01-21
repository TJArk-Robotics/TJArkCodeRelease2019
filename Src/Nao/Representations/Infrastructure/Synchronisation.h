#pragma once

#include <pthread.h>

class Synchronisation
{
public:
    pthread_mutex_t button;
    pthread_mutex_t serialization;
};
