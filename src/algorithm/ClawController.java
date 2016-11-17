/**
 * The ClawController class contains methods
 * used to lift and activate the claw
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 */

package algorithm;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import component.Constants;

public class ClawController {
	
	EV3LargeRegulatedMotor clawLift, clawClose;

	
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
		lift(-Constants.CLAW_LIFT_FULL);
		// grab block
		grab();
		// lift block up
		lift(Constants.CLAW_LIFT_FULL);
	}
	
	/**
	 * Puts down a blue foam block
	 */
	public void placeBlock(boolean stackBlock)
	{
		if (stackBlock) {
			lift(Constants.CLAW_LIFT_TWO_BLOCK);
			release();
			lift(Constants.CLAW_LIFT_TWO_BLOCK - Constants.CLAW_LIFT_FULL);
		} else {
			lift(Constants.CLAW_LIFT_ONE_BLOCK);
			release();
			lift(Constants.CLAW_LIFT_ONE_BLOCK - Constants.CLAW_LIFT_FULL);
		}
	}
	
	/**
	 * Closes the claw 
	 */
	public void grab()
	{
		//TODO Test
		
		clawClose.rotate(-Constants.CLAW_CLOSE_ANGLE, false);
	}
	
	/**
	 * Opens the claw
	 */
	public void release()
	{
		
		//TODO Test
		
		clawClose.rotate(Constants.CLAW_CLOSE_ANGLE, false);
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
