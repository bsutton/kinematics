package robotics;

import static org.junit.Assert.*;

import org.junit.Test;

import robotics.meArm.MeArmServoCalculator;
import robotics.servos.ServoAngleToPwmCalculator;

public class ServoAngleToPwmCalculatorTest
{

	
	@Test
	public void testta()
	{
		MeArmServoCalculator arm = new MeArmServoCalculator();

		arm.setTurretAngle(Math.toRadians(-70));

		System.out.println( (int) arm.getTurretPwm());

		arm.setTurretAngle(Math.toRadians(80));

		System.out.println((int) arm.getTurretPwm());


		arm.setTurretAngle(0);

		System.out.println((int) arm.getTurretPwm());

	}
	@Test
	public void test()
	{
		ServoAngleToPwmCalculator c = new ServoAngleToPwmCalculator(180, 590,
				-75, 90);

		double pwmValue = c.getPwmValue(Math.toRadians(-75));
		System.out.println("pwm " + pwmValue);
		assertTrue(Math.abs(pwmValue - 180) < 2);

		pwmValue = c.getPwmValue(Math.toRadians(90));
		System.out.println("pwm " + pwmValue);
		assertTrue(Math.abs(pwmValue - 590) < 2);

	}
	
//	armBase = new ServoAngleToPwmCalculator(218, 560, -75, 45);
//	armCenter = new ServoAngleToPwmCalculator(245, 600, -90, 30);


	@Test
	public void test2()
	{
		ServoAngleToPwmCalculator c = new ServoAngleToPwmCalculator(218, 560, -75, 45);

		double pwmValue = c.getPwmValue(Math.toRadians(-75));
		System.out.println("pwm " + pwmValue);
		assertTrue(Math.abs(pwmValue - 218) < 2);

		pwmValue = c.getPwmValue(Math.toRadians(45));
		System.out.println("pwm " + pwmValue);
		assertTrue(Math.abs(pwmValue - 560) < 2);
		System.out.println(c.getPwmValue(0));

	}

	@Test
	public void test3()
	{
		ServoAngleToPwmCalculator c = new ServoAngleToPwmCalculator(110, 600, -70, 135);

		double pwmValue = c.getPwmValue(Math.toRadians(-70));
		System.out.println("pwm " + pwmValue);
		assertTrue(Math.abs(pwmValue - 110) < 2);

		pwmValue = c.getPwmValue(Math.toRadians(135));
		System.out.println("pwm " + pwmValue);
		assertTrue(Math.abs(pwmValue - 600) < 2);
		
		pwmValue = c.getPwmValue(Math.toRadians(106));
		System.out.println("pwm " + pwmValue);
		assertTrue(Math.abs(pwmValue - 600) < 2);

	}

}
