package robotics;

import static org.junit.Assert.assertTrue;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Test;

import robotics.meArm.MeArmKinematics;

public class MeArmTest
{

	@Test
	public void testArmExtensionInXYZ()
	{
		// y= forward

		MeArmKinematics arm = new MeArmKinematics();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		for (int y = 40; y < 100; y++)
			for (int x = -80; x < 80; x++)
				for (int z = 40; z < 100; z++)
				{
					Pose pose = new Pose(x, y, z, 0, 0, 0);

					arm.setPosition(pose);
					checkError(arm, pose);
				}

	}

	private void checkError(MeArmKinematics arm, Pose pose)
	{
		Vector3D endPoint = arm.getEndEffectorPose().getTransform().getVector();

		double xdiff = Math.abs(pose.getTransform().getVector().getX()
				- endPoint.getX());
		double ydiff = Math.abs(pose.getTransform().getVector().getY()
				- endPoint.getY());
		double zdiff = Math.abs(pose.getTransform().getVector().getZ()
				- endPoint.getZ());
		// System.out.println(pose.transform + " " + endPoint);
		assertTrue(xdiff < 3 && ydiff < 3 && zdiff < 3);
	}

}
