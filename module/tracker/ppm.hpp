#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

namespace Tracker
{
    class PPM
    {
    public:
        void init(double k, int planeWidth);
        int forward(int &pos_last, int &pos, int &pos_current);

    private:
        int planeWidth;
        double ki_2, ke_2, ki_1, ke_1;
        double paraCurve(double x, double k)
        {
            return -k * (x * x) + k;
        }
    };
}