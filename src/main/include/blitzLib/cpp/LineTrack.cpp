#include "LineTrack.hpp"

using namespace std;

double * Blitz::LineTrack::GetDirections()
{
    return directions;
}

void Blitz::LineTrack::Update()
{
    double x = alignX.GetDouble(0);
    double z = alignZ.GetDouble(0);

    directions[0] = 0;
    directions[1] = 0;
    directions[2] = 0;

    if (x > DEAD_ZONE)
    {
        directions[0] -= SPEED_H;
    }
	else if (x < -DEAD_ZONE)
	{
	    directions[0] += SPEED_H;
	}
    else if (z < -DEAD_ZONE_R)
    {
        directions[2] -= SPEED_R;
    }
    else if (z > DEAD_ZONE_R)
    {
        directions[2] += SPEED_R;
    }
    else if (z > -DEAD_ZONE_R && z < DEAD_ZONE_R)
    {
        directions[1] -= SPEED;
    }
}