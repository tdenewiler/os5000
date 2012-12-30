#include "os5000/timing.h"

Timing::Timing(float _period)
{
    period = _period;
    setup();
}

Timing::~Timing()
{
}

void Timing::setup()
{
    // Set timer to the current system time.
    set();

    // Initialize variables.
    expired = false;
}

bool Timing::checkExpired()
{
    // Declare variables.
    struct timeval t = {0, 0};
    int t1 = 0;
    int t2 = 0;

    // Get the current system time.
    gettimeofday(&t, NULL);

    // Convert times to microseconds.
    t1 = (s * 1000000) + us;
    t2 = (t.tv_sec * 1000000) + t.tv_usec;

    // Check to see if period has elapsed.
    if (t2 - t1 > (period * 1000000))
    {
        expired = true;
    }
    else
    {
        expired = false;
    }

    return expired;
}

void Timing::set()
{
    // Declare variables.
    struct timeval t = {0, 0};

    // Get the current system time.
    gettimeofday(&t, NULL);

    // Set timer to the current system time.
    s  = t.tv_sec;
    us = t.tv_usec;
}

int Timing::getDt()
{
    // Declare variables.
    struct timeval t = {0, 0};
    int elapsed_sec = 0;
    int elapsed_usec = 0;

    // Get the current system time.
    gettimeofday(&t, NULL);

    // Check to see which fraction of a second is larger.
    if (t.tv_usec > us)
    {
        elapsed_sec = s - t.tv_sec;
        elapsed_usec = 1000000 + us - t.tv_usec;
        // This should never happen. If it does then there is a problem with the system time.
        if (elapsed_sec < 0)
        {
            return TIMING_ERROR;
        }
    }
    else
    {
        elapsed_sec  = s - t.tv_sec;
        elapsed_usec = us - t.tv_usec;
    }

    return elapsed_sec * 1000000 + elapsed_usec;
}

float Timing::getDtS()
{
    // Declare variables.
    struct timeval t = {0, 0};
    int t1 = 0;
    int t2 = 0;

    // Get the current system time.
    gettimeofday(&t, NULL);

    // Convert times to microseconds.
    t1 = s * 1000000 + us;
    t2 = t.tv_sec * 1000000 + t.tv_usec;

    // Return time in seconds.
    return (float)(t2 - t1) / 1000000.;
}
