package robotics.arm4dof;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.Axis;
import robotics.Frame;
import robotics.Point3D;
import robotics.Pose;
import robotics.Transform;
import robotics.arm.ArmKinematics;
import robotics.arm.InvKinematics;

public class Arm4Dof extends ArmKinematics
{

	public static final int END_EFFECTOR_LENGTH = 10;
	public final Segment BASE_TO_SERVO;
	public final Segment TURRET_JOINT;
	public final Segment TURRET_TO_ARM_BASE;
	public final Segment BASE_JOINT;
	public final Segment ARM_SEGMENT1;
	public final Segment CENTER_JOINT;
	public final Segment ARM_SEGMENT2;
	public final Segment WRIST_JOINT;
	private Segment END_EFFECTOR;

	public Arm4Dof()
	{
		super(Frame.getWorldFrame(), new Pose(0, 0, 0, 0, 0, 0));

		BASE_TO_SERVO = addLink("Base to Servo", 0, 0, 20, 0, 0, 0);
		TURRET_JOINT = addJoint("", Axis.YAW, 0, 0, 0);
		TURRET_TO_ARM_BASE = addLink("Turret to arm Base", 0, 20, 20, 0, 0, 0);
		BASE_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		ARM_SEGMENT1 = addLink("Arm segment 1", 0, 0, 81, 0, 0, 0);
		CENTER_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		ARM_SEGMENT2 = addLink("Arm segment 2", 0, 0, 81, 0, 0, 0);
		WRIST_JOINT = addJoint("", Axis.PITCH, 0, 0, 0);
		END_EFFECTOR = addLink("EndEffector", 0, 0, END_EFFECTOR_LENGTH, 0, 0,
				0);

		setInvKinematics(getInvKinematics());
	}

	private InvKinematics getInvKinematics()
	{
		return new InvKinematics()
		{

			public void determine(Pose tipPose)
			{

				// determine and set turret angle

				resetJointsToZero();

				double y = tipPose.getY();
				double x = tipPose.getX();

				double turretAngle = Math.atan2(x, y);

				setJointAngle(TURRET_JOINT, turretAngle);
				Vector3D armBase = getSegmentPose(BASE_JOINT).getTransform()
						.getVector();

				Pose endEffectorPose = getEndEffectorPoseFromTipPose(tipPose,
						turretAngle);

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
						new Point3D(getFrame(), armBase.getX(), armBase.getY(),
								0)).getDistance();

				double baseAngle = Math.atan2(x, z) - (midArmAngle / 2.0);

				setJointAngle(BASE_JOINT, baseAngle);
				setJointAngle(CENTER_JOINT, midArmAngle);

				// compensate the desired tip pose angle for the pose of the
				// wrist angle
				double wristAngle = tipPose.getXAngle()
						- getSegmentPose(WRIST_JOINT).getXAngle();
				setJointAngle(WRIST_JOINT, wristAngle);

			}

			private Pose getEndEffectorPoseFromTipPose(Pose tipPose,
					double turretAngle)
			{
				// get the vector representing the end effector
				Vector3D tipVector = getLink(END_EFFECTOR).getTransform()
						.getVector();

				// create a rotation for the desired x rotation for the end
				// effector plus the required z rotation to match the turret
				// angle of the arm base
				Rotation turretCorrectedRotation = new Rotation(
						RotationOrder.XYZ, tipPose.getXAngle(), 0, turretAngle);

				// rotate the tip vector
				Vector3D turretCorrectedTipVector = turretCorrectedRotation
						.applyInverseTo(tipVector);

				// subtract the tip vector from the tipPose to get the root of
				// the end effector
				Point3D rootOfEndEffector = tipPose.getTransform().subtract(
						turretCorrectedTipVector);

				// construct a new pose for the root of the end effector
				Pose tipPose1 = new Pose(rootOfEndEffector.getPoint(),
						turretCorrectedRotation);

				return tipPose1;
			}

		};

	}

}
