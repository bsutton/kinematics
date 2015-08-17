package robotics.servos;


public class ServoAngleToPwmCalculator
{

	private double m;
	private double c;
	private double min;
	private double max;

	public ServoAngleToPwmCalculator(double minPwm, double maxPwm, double minAngle,
			double maxAngle)
	{
		double minAngleR = Math.toRadians(minAngle);
		double maxAngleR = Math.toRadians(maxAngle);

		min = minPwm;
		max = maxPwm;
		// y = pwm

		// x = angle

		// y = m*x+c;

		// y-c = m*x

		// m = (y-c)/x ... m = deltaY/deltaX;

		// y-(m*x) = c

		m = (maxPwm - minPwm) / (maxAngleR - minAngleR);

		c = minPwm - (minAngleR * m);
	}
	
	public ServoAngleToPwmCalculator(ServoConfig servoConfig)
	{
		this(servoConfig.getMinPwm(), servoConfig.getMaxPwm(), servoConfig.getMinAngle(),
			servoConfig.getMaxAngle());
	}


	public double getPwmValue(double angleX)
	{
		double pwm = (m * angleX) + c;
		return Math.max(min, Math.min(pwm, max));
	}
}
