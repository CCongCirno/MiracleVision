#include "ppm.hpp"
namespace Tracker
{
    void PPM::init(double k, int planeWidth)
    {
        this->planeWidth = planeWidth;
        ki_2 = (k + 2) * (k - 1) / 4;
        ke_2 = (k + 2) * (k + 1) / 4;
        ki_1 = k * (k + 1) / 4;
        ke_1 = ke_2;
    }

    int PPM::forward(int &pos_last, int &pos, int &pos_current)
    {
        double potential[2048];
        int max_sum = 0;
        double maxn = 0;
        double distance;
        double excitatoryWeight1;
        double inhibitedWeight;
        double excitatoryWeight3;
        for (int i = 0; i < planeWidth; ++i)
        {
            distance = std::abs(pos_last - i) * 1.0 / planeWidth;
            excitatoryWeight1 = paraCurve(distance, ki_1);
            distance = std::abs(pos - i) * 1.0 / planeWidth;
            inhibitedWeight = paraCurve(distance, ke_1 + ki_2);
            distance = std::abs(pos_current - i) * 1.0 / planeWidth;
            excitatoryWeight3 = paraCurve(distance, ke_2);
            potential[i] = excitatoryWeight1 - inhibitedWeight + excitatoryWeight3;
            if (i == 0)
            {
                maxn = potential[0];
                max_sum = 0;
            }
            else if (maxn < potential[i])
            {
                maxn = potential[i];
                max_sum = i;
            }
        }
        return max_sum;
    }
}