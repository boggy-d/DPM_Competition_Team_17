/**
 * The ClawController class contains methods
 * used to lift and activate the claw
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 */

package algorithm;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ClawController {
	
	EV3LargeRegulatedMotor clawLift, clawClose;
	
	private final int CLAW_CLOSE_ANGLE = 145;
	
	public ClawController(EV3LargeRegulatedMotor clawLift, EV3LargeRegulatedMotor clawClose) {
		this.clawLift = clawLift;
		this.clawClose = clawClose;
	}
	
	/**
	 * Closes the claw 
	 */
	public void grab()
	{
		//TODO Test
		
		clawClose.rotate(-CLAW_CLOSE_ANGLE, false);
	}
	
	/**
	 * Opens the claw
	 */
	public void release()
	{
		
		//TODO Test
		
		clawClose.rotate(CLAW_CLOSE_ANGLE, false);
	}
	
	/**
	 * Lift the claw to the desired angle
	 * @param angle the angle to lift the claw at
	 */
	public void lift(int angle)
	{
		//TODO Test
		
		clawLift.rotate(angle, false);
	}
}
