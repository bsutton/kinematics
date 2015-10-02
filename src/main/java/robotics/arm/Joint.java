package robotics.arm;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

import robotics.Axis;

/**
 * Used by Kinematics to make calculations about a definition as a pose is
 * calculated.
 * 
 * @author bsutton
 *
 */
class Joint extends Link
{
	private double[] angles;

	private Axis axis;

	private double currentAngle;

	/**
	 * a joint is a mutable. and used by ArmKinematics to represent the
	 * mathematical state of the arm
	 * 
	 * @param name
	 * @param axis
	 * @param roll
	 * @param pitch
	 * @param yaw
	 */
	Joint(String name, Axis axis, double roll, double pitch, double yaw)
	{
		super(name, 0, 0, 0, roll, pitch, yaw);
		this.axis = axis;
		this.angles = getRotation().getAngles(RotationOrder.XYZ);
		currentAngle = 0;
	}

	void setAngle(double angle)
	{
		currentAngle = angle;
		setRotation(new Rotation(RotationOrder.XYZ, angles[0]
				+ axis.getRotatedAngle(angle, Axis.PITCH),

		angles[1] + axis.getRotatedAngle(angle, Axis.ROLL),

		angles[2] + axis.getRotatedAngle(angle, Axis.YAW)));

	}

	double getSetAngle()
	{
		return currentAngle;
	}

}
