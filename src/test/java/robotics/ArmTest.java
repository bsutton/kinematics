package robotics;

import static org.junit.Assert.assertTrue;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3DFormat;
import org.junit.Test;

import robotics.arm.ArmKinematics;

public class ArmTest
{

	// JointDefinition TURRET_JOINT_DEF = new JointDefinition("Turret",
	// Axis.YAW);
	// JointDefinition BASE_JOINT_DEF = new JointDefinition("Arm Base",
	// Axis.PITCH);
	// JointDefinition CENTRE_JOINT_DEF = new JointDefinition("Arm Center",
	// Axis.PITCH);
	// JointDefinition WRIST_JOINT_DEF = new JointDefinition("Wrist",
	// Axis.PITCH);

	// @Test
	// public void testTurretAngle()
	// {
	// // y= forward
	//
	// Arm arm = defineArm();
	//
	// // all angles zero
	// // Pose pose = new Pose(0, 20, 202, 0, 0, 0);
	//
	// int x = 0;
	// int z = 40;
	// for (int y = 40; y < 150; y++)
	// {
	// Pose pose = new Pose(x, y, z, 0, 0, 0);
	//
	// arm.setPosition(pose);
	// Pose endPoint = arm.getEndEffectorPose();
	//
	// double xdiff = Math.abs(pose.transform.transform.getX()
	// - endPoint.transform.transform.getX());
	// double ydiff = Math.abs(pose.transform.transform.getY()
	// - endPoint.transform.transform.getY());
	// double zdiff = Math.abs(pose.transform.transform.getZ()
	// - endPoint.transform.transform.getZ());
	//
	// System.out.println(arm.setJointAngle("Turret").setJointAngleAngle());
	// System.out.println(arm.setJointAngle("Arm Base").setJointAngleAngle());
	// System.out.println(arm.setJointAngle("Arm Center").setJointAngleAngle());
	// System.out.println(x + " " + y + " " + z + " got " + endPoint);
	// System.out.println(xdiff + " " + ydiff + " " + zdiff);
	// assertTrue(xdiff < 2 && ydiff < 2 && zdiff < 2);
	// }
	//
	// }

	@Test
	public void test1()
	{
		Vector3D v1 = new Vector3D(1, 0, 0);
		// rotate around Y axis, X becomes Z
		Rotation r1 = new Rotation(RotationOrder.XYZ, 0, Math.PI / 2, 0);
		Vector3D v2 = new Vector3D(0, 1, 0);

		Vector3D result = r1.applyInverseTo(v1).add(v2);

		Vector3DFormat format = new Vector3DFormat();

		System.out.println(format.format(result));
	}

	@Test
	public void test2()
	{
		Vector3D v1 = new Vector3D(1, 0, 0);
		// rotate around Y axis, X becomes Z
		Rotation r1 = new Rotation(RotationOrder.XYZ, 0, Math.PI / 2, 0);
		Vector3D v2 = new Vector3D(0, 1, 0);

		Vector3D result = r1.applyInverseTo(v1).add(v2);

		Vector3DFormat format = new Vector3DFormat();

		System.out.println(format.format(result));
	}

	@Test
	public void testJoint1()
	{
		TestArmKinematics arm = defineArm();
		arm.setJointAngle(arm.TURRET_JOINT, 0);
		Pose pose = new Pose(0, 20,
				202 + TestArmKinematics.END_EFFECTOR_LENGTH, 0, 0, 0);

		checkError(arm, pose);

		arm.setJointAngle(arm.TURRET_JOINT, Math.PI / -2);
		pose = new Pose(-20, 0, 202 + TestArmKinematics.END_EFFECTOR_LENGTH, 0,
				0, 0);
		checkError(arm, pose);

		arm.setJointAngle(arm.TURRET_JOINT, Math.PI / 2);
		pose = new Pose(20, 0, 202 + TestArmKinematics.END_EFFECTOR_LENGTH, 0,
				0, 0);
		checkError(arm, pose);

	}

	@Test
	public void testJoint2()
	{
		TestArmKinematics arm = defineArm();
		arm.setJointAngle(arm.BASE_JOINT, 0);
		Pose pose = new Pose(0, 20,
				202 + TestArmKinematics.END_EFFECTOR_LENGTH, 0, 0, 0);
		checkError(arm, pose);

		arm.setJointAngle(arm.BASE_JOINT, Math.PI / 2);
		pose = new Pose(0, 182 + TestArmKinematics.END_EFFECTOR_LENGTH, 40, 0,
				0, 0);
		checkError(arm, pose);

		arm.setJointAngle(arm.BASE_JOINT, Math.PI / -2);
		pose = new Pose(0, -142 - TestArmKinematics.END_EFFECTOR_LENGTH, 40, 0,
				0, 0);
		checkError(arm, pose);

	}

	@Test
	public void testTurretAngleDoesntAffectZ()
	{
		TestArmKinematics arm = defineArm();
		Double z = arm.getEndEffectorPose().getZ();
		for (double a = -Math.PI; a < Math.PI; a += Math.PI / 10.0)
		{
			arm.setJointAngle(arm.TURRET_JOINT, a);
			System.out.println(z + " " + arm.getEndEffectorPose().getZ());
			assertTrue(Math.abs(arm.getEndEffectorPose().getZ() - z) < .1);

		}
	}

	@Test
	public void testArmBaseAngleDoesntAffectX()
	{
		TestArmKinematics arm = defineArm();
		Double x = arm.getEndEffectorPose().getX();
		for (double a = -Math.PI; a < Math.PI; a += Math.PI / 10.0)
		{
			arm.setJointAngle(arm.BASE_JOINT, a);
			System.out.println(x + " " + arm.getEndEffectorPose().getX());
			assertTrue(Math.abs(arm.getEndEffectorPose().getX() - x) < .1);

		}
	}

	@Test
	public void testArmMidAngleDoesntAffectX()
	{
		TestArmKinematics arm = defineArm();
		Double x = arm.getEndEffectorPose().getX();
		for (double a = -Math.PI; a < Math.PI; a += Math.PI / 10.0)
		{
			arm.setJointAngle(arm.CENTER_JOINT, a);
			System.out.println(x + " " + arm.getEndEffectorPose().getX());
			assertTrue(Math.abs(arm.getEndEffectorPose().getX() - x) < .1);

		}
	}

	@Test
	public void testArmExtensionInZ()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		int x = 0;
		int y = 40;
		for (int z = 40; z < 150; z++)
		{
			Pose pose = new Pose(x, y, z, 0, 0, 0);

			arm.setPosition(pose);
			checkError(arm, pose);
		}

	}

	@Test
	public void testArmExtensionInXYZ()
	{
		// y= forward

		ArmKinematics arm = defineArm();

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
	
	

	@Test
	public void testArmExtensionInXYZplusPose()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		for (int y = 40; y < 100; y++)
			for (int x = -80; x < 80; x++)
				for (int z = 40; z < 100; z++)
					for (int p = 90; p < 180; p++)
					{
						//System.out.println(x + " " + y + " " + z + " " + p);
						Pose pose = new Pose(x, y, z, Math.toRadians(p), 0, 0);

						arm.setPosition(pose);
						checkError(arm, pose);
					}

	}

	@Test
	public void testArmExtensionInY()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		int x = 0;
		int z = 40;
		for (int y = 40; y < 150; y++)
		{
			System.out.println(y);
			Pose pose = new Pose(x, y, z, 0, 0, 0);

			arm.setPosition(pose);
			checkError(arm, pose);
		}
	}

	@Test
	public void testArmExtensionInX()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		// Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		int y = 40;
		int z = 40;
		for (int x = -80; x < 80; x++)
		{
			Pose pose = new Pose(x, y, z, 0, 0, 0);

			arm.setPosition(pose);
			checkError(arm, pose);
		}

	}

	private TestArmKinematics defineArm()
	{

		return new TestArmKinematics();
	}

	@Test
	public void testArmBase90Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// arm base 90 degrees
		Pose pose = new Pose(0, 181.95 - TestArmKinematics.END_EFFECTOR_LENGTH,
				40, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testAllAnglesZero()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// all angles zero
		arm.resetJointsToZero();

		Pose pose = new Pose(0, 20, 202, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testTurret45Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// turret 90 degrees
		Pose pose = new Pose(14, 14, 101.00, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testTurret22Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// turret 90 degrees
		Pose pose = new Pose(14, 34, 101.00, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	@Test
	public void testTurret90Degrees()
	{
		// y= forward

		ArmKinematics arm = defineArm();

		// turret 90 degrees
		Pose pose = new Pose(19, 0, 150, 0, 0, 0);

		arm.setPosition(pose);

		checkError(arm, pose);

	}

	private void checkError(ArmKinematics arm, Pose pose)
	{
		Vector3D endPoint = arm.getEndEffectorPose().getTransform().getVector();

		double xdiff = Math.abs(pose.getTransform().getVector().getX()
				- endPoint.getX());
		double ydiff = Math.abs(pose.getTransform().getVector().getY()
				- endPoint.getY());
		double zdiff = Math.abs(pose.getTransform().getVector().getZ()
				- endPoint.getZ());
		// System.out.println(pose.transform + " " + endPoint);
		if (!((xdiff < 3 && ydiff < 3 && zdiff < 3)))
		{
			System.out.println(pose.getTransform() + " != " + endPoint);
		}
		assertTrue(xdiff < 3 && ydiff < 3 && zdiff < 3);
	}

}
