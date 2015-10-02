package trajectory;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class TrajectoryStep
{

	public TrajectoryStep(double time, double x, double y, double z)
	{
		stepOffsetMs = (long) time;
		location = new Vector3D(x, y, z);
	}

	long stepOffsetMs;

	Vector3D location;
}
