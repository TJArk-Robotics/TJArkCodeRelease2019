/*
 * @data: 2019-03-30
 * @author: xuzihan
 * @file: battery.cpp
 * 
 * ################## Build #########################
 * Using Lola_connector project to build this program.
 * qibuild is also available.
 * ##################################################
 * 
 * Get information of Robot battery status and charge value. This information 
 * will show in RobotView box when using bush
 * In Nao V5, Bhuman use a python scripts to get this information, but in V6, when 
 * we connect to lola, naoqi cann't provides this information any more, so we get this information
 * from the shared memory.
 */

#include "LoLAConnector/bhuman.h"

#include <iostream>
#include <stdint.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <memory.h>
#include <stdio.h>
#include <sys/io.h>
#include <unistd.h>
#include <cmath>

using namespace std;

void readGryoData(LBHData *data);
bool checkData(float x, float y, float z);

int main(int argc, char **argv)
{
    LBHData *data;
    sem_t *sem = SEM_FAILED;
    int frameDrops = 7;

    // create shared memory
    int fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
    if (fd == -1)
        return -1;

    // open semaphore
    sem = sem_open(LBH_SEM_NAME, O_RDWR, S_IRUSR | S_IWUSR, 0);
    if (sem == SEM_FAILED)
    {
        close(fd);
        fd = -1;
        return -2;
    }

    data = (LBHData *)mmap(nullptr, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED)
    {
        cerr << "battery: mmap" << endl;
        return -3;
    }

    int cnt = 0;
    bool pass = false;
    int correctData = 0;
    while (cnt++ < 10)
    {
        try
        {
            // get sensor data
            float gyroX;
            float gyroY;
            float gyroZ;

            gyroX = data->sensors[0][gyroXSensor];
            gyroY = data->sensors[0][gyroYSensor];
            gyroZ = data->sensors[0][gyroZSensor];

            cout << "gyroX ---- >>>> " << gyroX << endl;
            cout << "gyroY ---- >>>> " << gyroY << endl;
            cout << "gyroZ ---- >>>> " << gyroZ << endl << endl;

            if (checkData(gyroX, gyroY, gyroZ))
            {
                correctData++;
            }

            // readGryoData(data);
            //

            // raise the semaphore
            if (sem != SEM_FAILED)
            {
                int sval;
                if (sem_getvalue(sem, &sval) == 0)
                {
                    if (sval < 1)
                    {
                        sem_post(sem);
                        frameDrops = 0;
                    }
                    else
                    {
                        ++frameDrops;
                    }
                }
            }
        }
        catch (string &e)
        {
            cerr << e << endl;
        }
        usleep(100 * 1000);
    }

    if (correctData == 10)
    {
        pass = true;
    }

    if (pass)
    {
        cout << "Gyro Data is correct." << endl;
    }
    else
    {
        cout << "Gyro Data is error! Please do as follows:\n";
        cout << "Use 'halt' command to turn off robot and then press the button to start robot. ";
    }
    

    // clean up
    if (data != MAP_FAILED)
    {
        munmap(data, sizeof(LBHData));
        data = (LBHData *)MAP_FAILED;
    }
    if (fd != -1)
    {
        close(fd);
        fd = -1;
    }
    if (sem != SEM_FAILED)
    {
        sem_close(sem);
        sem = SEM_FAILED;
    }

    return 0;
}

void readGryoData(LBHData *data)
{
    float gyroX;
    float gyroY;
    float gyroZ;

    gyroX = data->sensors[0][gyroXSensor];
    gyroY = data->sensors[0][gyroYSensor];
    gyroZ = data->sensors[0][gyroZSensor];

    cout << "gyroX ---- >>>> " << gyroX << endl;
    cout << "gyroY ---- >>>> " << gyroY << endl;
    cout << "gyroZ ---- >>>> " << gyroZ << endl
         << endl;
}

bool checkData(float x, float y, float z)
{
    float err = abs(x - 0) + abs(y - 0) + abs(z - 0);
    return (err < 5);
}