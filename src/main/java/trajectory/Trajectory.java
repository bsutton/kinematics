package trajectory;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Trajectory
{

	List<TrajectoryStep> createStraightPath(Vector3D start, Vector3D end,
			int steps, int durationMs)
	{
		double xChange = end.getX() - start.getX();
		double yChange = end.getY() - start.getY();
		double zChange = end.getZ() - start.getZ();

		List<TrajectoryStep> trajectorySteps = new LinkedList<>();

		double interval = durationMs / steps;
		for (double i = 1; i <= steps; i++)
		{
			double portion = i / steps;
			double x = start.getX() + (xChange * portion);
			double y = start.getY() + (yChange * portion);
			double z = start.getZ() + (zChange * portion);
			trajectorySteps.add(new TrajectoryStep(i * interval, x, y, z));
		}

		return trajectorySteps;

	}
}
