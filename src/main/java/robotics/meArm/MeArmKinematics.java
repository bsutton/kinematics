package robotics.meArm;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.Axis;
import robotics.Frame;
import robotics.Point3D;
import robotics.Pose;
import robotics.Transform;
import robotics.arm.ArmKinematics;
import robotics.arm.InvKinematics;

public class MeArmKinematics extends ArmKinematics
{

	public final Segment BASE_TO_SERVO;
	public final Segment TURRET_JOINT;
	public final Segment TURRET_TO_ARM_BASE;
	public final Segment BASE_JOINT;
	public final Segment ARM_SEGMENT1;
	public final Segment CENTER_JOINT;
	public final Segment ARM_SEGMENT2;
	public final Segment WRIST_JOINT;

	public MeArmKinematics()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		// define our arms characteristics
		BASE_TO_SERVO = addLink("Base to Servo", 0, 0, 20, 0, 0, 0);
		TURRET_JOINT = addJoint("", Axis.YAW, 0, 0, 0);
		TURRET_TO_ARM_BASE = addLink("Turret to arm Base", 0, 20, 20, 0, 0, 0);
		BASE_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		ARM_SEGMENT1 = addLink("Arm segment 1", 0, 0, 81, 0, 0, 0);
		CENTER_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		ARM_SEGMENT2 = addLink("Arm segment 2", 0, 0, 81, 0, 0, 0);
		WRIST_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);

		setInvKinematics(getInvKinematics());
	}

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(Pose endEffectorPose)
			{

				// determine and set turret angle

				double y = endEffectorPose.getY();
				double x = endEffectorPose.getX();

				double turretAngle = Math.atan2(x, y);

				setJointAngle(TURRET_JOINT, turretAngle);
				Vector3D armBase = getSegmentPose(BASE_JOINT)
						.getTransform().getVector();

				// calculate distance between armBase and wrist
				double extend = Vector3D.distance(armBase, endEffectorPose
						.getTransform().getVector());
				// double extend = new Transform(endPoint,
				// armBase).getDistance();

				// calculate angle of the bend in the arm to give the desired
				// length, assuming the two links are of the same length.
				double linkLength = 81.0;
				double midArmAngle = Math.acos(((extend / 2.0) / linkLength)) * 2.0;

				// midArmAngle -= Math.PI;

				// calculate angle for armBase
				// atan(changeInXY/changeInZ)

				double z = endEffectorPose.getZ() - armBase.getZ();
				x = new Transform(new Point3D(getFrame(),
						endEffectorPose.getX(), endEffectorPose.getY(), 0),
						new Point3D(getFrame(), armBase.getX(), armBase
								.getY(), 0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				setJointAngle(BASE_JOINT, baseAngle);
				setJointAngle(CENTER_JOINT, midArmAngle);

			}

		};
	}

}
