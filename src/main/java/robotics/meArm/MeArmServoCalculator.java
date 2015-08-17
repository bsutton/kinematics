package robotics.meArm;

import robotics.servos.ServoAngleToPwmCalculator;

/**
 * extends the MeArmKinematics, adding pwm calculations on top of the joint
 * angle calculations
 * 
 * @author rsutton
 *
 */
public class MeArmServoCalculator extends MeArmKinematics
{

	private ServoAngleToPwmCalculator turret;
	private ServoAngleToPwmCalculator armBase;
	private ServoAngleToPwmCalculator armCenter;

	public MeArmServoCalculator()
	{
		turret = new ServoAngleToPwmCalculator(180, 590, -75, 90);

		armBase = new ServoAngleToPwmCalculator(218, 560, -75, 45);

		armCenter = new ServoAngleToPwmCalculator(110, 600, -70, 135);
	}

	public double getTurretPwm()
	{
		return turret.getPwmValue(getComputedJointAngle(TURRET_JOINT));
	}

	public double getArmBasePwm()
	{
		Double jointAngle = -1.0 * getComputedJointAngle(BASE_JOINT);
		double pwmValue = armBase.getPwmValue(jointAngle);
		System.out.println("Base angle: " + Math.toDegrees(jointAngle)
				+ " pwm: " + pwmValue);
		return pwmValue;
	}

	public double getArmCenterPwm()
	{
		// arm base and arm centre are actually parallel joints, so we sum them
		// to set the servo angle for the enter join

		System.out.println("Center: "
				+ Math.toDegrees(getComputedJointAngle(CENTER_JOINT)));
		double d = (getComputedJointAngle(CENTER_JOINT) * -1.0) + Math.PI;
		System.out.println(Math.toDegrees(d));
		double angleX = (-1.0 * getComputedJointAngle(BASE_JOINT)) + d;

		if (angleX > Math.PI)
		{
			System.out.println("Move into correct range (- 2PI)");
			angleX -= Math.PI * 2.0;
		}
		double pwmValue = armCenter.getPwmValue(angleX);
		System.out.println("Center angle: " + Math.toDegrees(angleX) + " pwm: "
				+ pwmValue);
		return pwmValue;
	}

	public void setTurretAngle(double i)
	{
		setJointAngle(TURRET_JOINT,i);

	}

	public void setArmBaseAngle(double radians)
	{
		setJointAngle(BASE_JOINT,radians);

	}

	public void setArmCenterAngle(double radians)
	{
		setJointAngle(CENTER_JOINT,radians);

	}
}
