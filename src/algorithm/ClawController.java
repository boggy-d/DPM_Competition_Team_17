/**
 * The ClawController class contains methods
 * used to lift and activate the claw
 * 
 * @author Bogdan Dumitru
 * @author Eva Suska
 * @version 0.1.0
 */

package algorithm;

import component.ActionController;
import component.Constants;


public class ClawController {
	
	private boolean isBlockGrabbed;
	
	/**
	 * Class constructor initializing the block grabbed flag
	 */
	public ClawController() {
		
		isBlockGrabbed = false;
	}
	
	/**
	 * Picks up a blue foam block
	 */
	public void pickUpBlock()
	{
		ActionController.stopMotors();
		ActionController.goForward(-5, Constants.FORWARD_SPEED);
		// open claw
		release();
		// put claw down
		lift(Constants.CLAW_LIFT_FULL);
		// grab block
		grab();
		// lift block up
		lift(-Constants.CLAW_LIFT_FULL);
		this.setBlockGrabbed(true);
	}
	
	/**
	 * Puts down a blue foam block
	 * @param stackBlock <code>true</code> if the robot is stacking the second floor, <code>false</code>otherwise
	 */
	public void placeBlock(boolean stackBlock)
	{
		if (stackBlock) {
			// put claw down
			lift(Constants.CLAW_LIFT_FULL - Constants.CLAW_STACK);
			release();
			// lift block up
			lift(-(Constants.CLAW_LIFT_FULL- Constants.CLAW_STACK));
			// close claw
			grab();
		} else {
			// put claw down
			lift(Constants.CLAW_LIFT_FULL);
			release();
			// lift block up
			lift(-Constants.CLAW_LIFT_FULL);
			// close claw
			grab();
		}
		this.setBlockGrabbed(false);
	}
	
	/**
	 * Accessor method that returns if the block currently has a block captured or not
	 * @return <code>true</code> if the claw is holding a block, <code>false</code> otherwise
	 */
	public boolean isBlockGrabbed()
	{
		return isBlockGrabbed;
	}
	
	/**
	 * Mutator method that sets the <code>isBlockGrabbed</code> flag
	 * @param blockGrabbed the new state of the claw (holding a block or not)
	 */
	public void setBlockGrabbed(boolean blockGrabbed)
	{
		this.isBlockGrabbed = blockGrabbed;
	}
	
	/**
	 * Closes the claw 
	 */
	public void grab()
	{
		//TODO Test
		
		Constants.clawClose.rotate(-Constants.CLAW_CLOSE_ANGLE, false);
	}
	
	/**
	 * Opens the claw
	 */
	public void release()
	{
		
		//TODO Test
		
		Constants.clawClose.rotate(Constants.CLAW_OPEN_ANGLE, false);
	}
	
	/**
	 * Lift the claw to the desired angle
	 * @param angle the angle to lift the claw at
	 */
	public void lift(int angle)
	{
		//TODO Test
		
		Constants.clawLift.rotate(angle, false);
	}
	
}
