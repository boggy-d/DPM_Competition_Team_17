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
	
	private final int CLAW_LIFT_FULL = 775;
	private final int CLAW_LIFT_ONE_BLOCK = -700;
	private final int CLAW_LIFT_TWO_BLOCK = 600;
	private final int CLAW_LIFT_THREE_BLOCK = 500;
	private final int CLAW_CLOSE_ANGLE = 145;
	
	public ClawController(EV3LargeRegulatedMotor clawLift, EV3LargeRegulatedMotor clawClose) {
		this.clawLift = clawLift;
		this.clawClose = clawClose;
	}
	
	//TODO fix constants for the lifting
	/**
	 * Picks up a blue foam block
	 */
	public void pickUpBlock()
	{
		// put claw down
		lift(-CLAW_LIFT_FULL);
		// grab block
		grab();
		// lift block up
		lift(CLAW_LIFT_FULL);
	}
	
	/**
	 * Puts down a blue foam block
	 */
	public void placeBlock(boolean stackBlock)
	{
		if (stackBlock) {
			lift(CLAW_LIFT_TWO_BLOCK);
			release();
			lift(CLAW_LIFT_TWO_BLOCK - CLAW_LIFT_FULL);
		} else {
			lift(CLAW_LIFT_ONE_BLOCK);
			release();
			lift(CLAW_LIFT_ONE_BLOCK - CLAW_LIFT_FULL);
		}
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
