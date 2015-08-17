package robotics.servos;

/**
 * Attempt to abstract out the definition of a motor to make it easier to 
 * create specific robot implementations
 * 
 * A motor powers a joint
 */
public interface ServoConfig
{
	String getName();
	
	double getMaxPwm();

	double getMinPwm();

	double getMinAngle();

	double getMaxAngle();
}
