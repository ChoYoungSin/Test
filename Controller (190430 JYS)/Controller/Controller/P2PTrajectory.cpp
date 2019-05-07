
#include "TcPch.h"
#pragma hdrstop
#include "P2PTrajectory.h"



P2PTrajectory::P2PTrajectory()
{
}


P2PTrajectory::~P2PTrajectory()
{
}
// Point to point movement with trapezoidal velocity profile
bool P2PTrajectory::PointToPointInit(float destinationDeg, float speedDeg, float accDeg)
{
	speedProfileDeg = speedDeg;
	accProfileDeg = accDeg;


	startPositionDeg = positionStateRad * RAD_TO_DEG;
	displacementDeg = destinationDeg - startPositionDeg;

	if (displacementDeg > 0)
		dirDisplacement = 1;
	else
		dirDisplacement = -1;

	// Trapezoidal velocity profile
	if (fabs_(displacementDeg) > sqr_(speedProfileDeg) / fabs_(accProfileDeg))
	{
		modeProfile = 1;

		tAcc = fabs_(speedProfileDeg / accProfileDeg);
		tConst = fabs_(displacementDeg / speedProfileDeg);
		tDest = tConst + tAcc;

		dAcc = dirDisplacement * 0.5 * fabs_(accProfileDeg) * sqr_(tAcc);
		dConst = dirDisplacement * fabs_(speedProfileDeg) * (tConst - tAcc) + dAcc;
	}
	// Triangular velocity profile
	else
	{
		modeProfile = 2;

		maxVelProfile = sqrt_(fabs_(accProfileDeg * displacementDeg));

		tAcc = sqrt_(fabs_(displacementDeg / accProfileDeg));
		tDest = 2 * tAcc;

		dAcc = dirDisplacement * 0.5 * fabs_(displacementDeg);
	}

	return true;
}

bool P2PTrajectory::PointToPointPlay(float time)
{
	float	t;

	switch (modeProfile)
	{
		// Trapezoidal velocity profile	
	case 1:
		if (time < tAcc)
		{
			t = time;
			positionTargetRad = DEG_TO_RAD * (dirDisplacement * 0.5 * accProfileDeg * sqr_(t) + startPositionDeg);
			return false;
		}
		else if (time < tConst)
		{
			t = time - tAcc;
			positionTargetRad = DEG_TO_RAD * (dirDisplacement * speedProfileDeg * t + dAcc + startPositionDeg);
			return false;
		}
		else if (time < tDest)
		{
			t = time - tConst;
			positionTargetRad = DEG_TO_RAD * (-dirDisplacement * 0.5 * accProfileDeg * sqr_(t)
				+ dirDisplacement * speedProfileDeg * t + dConst + startPositionDeg);
			return false;
		}
		else
		{
			positionTargetRad = DEG_TO_RAD * (displacementDeg + startPositionDeg);
			return true;
		}
		// Triangular velocity profile
	case 2:
		if (time < tAcc)
		{
			t = time;
			positionTargetRad = DEG_TO_RAD * (dirDisplacement * 0.5 * accProfileDeg * sqr_(t) + startPositionDeg);
			return false;
		}
		else if (time < tDest)
		{
			t = time - tAcc;
			positionTargetRad = DEG_TO_RAD * (-dirDisplacement * 0.5 * accProfileDeg * sqr_(t)
				+ dirDisplacement * maxVelProfile * t + dAcc + startPositionDeg);
			return false;
		}
		else
		{
			positionTargetRad = DEG_TO_RAD * (displacementDeg + startPositionDeg);
			return true;
		}
	}
}

bool P2PTrajectory::TrajectoryPlay(float targetDeg)
{
	positionTargetRad = targetDeg * DEG_TO_RAD;

	return true;
}

bool P2PTrajectory::TrajectoryPlayRad(float targetRad)
{
	positionTargetRad = targetRad;

	return true;
}
bool P2PTrajectory::HardStop()
{
	positionTargetRad = positionStateRad;

	return true;
}
