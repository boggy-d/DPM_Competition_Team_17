/**
 * The Builder class contains methods related to the competition
 * builder role i.e. navigating to the green zone, picking up blocks, and stacking blocks
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 */


package algorithm;

public class Builder {

	/**
	 * Navigates to the wifi assigned Green Zone
	 * while avoiding blocks
	 * 
	 * @see ActionController
	 * @see Navigator
	 * @see ObstacleAvoidance
	 */
	public void goToGreen()
	{
		
	}
	
	/**
	 * Uses the LightPoller to detect the color of the
	 * floor in addition to checking if the
	 * wifi coordinates match. Returns if it is green or not.
	 * 
	 * @return <code>true</code> if the robot reached a green zone, otherwise return <code>false</code>
	 * 
	 * @see ActionController
	 * @see LightPoller
	 */
	public boolean isGreen()
	{
		return false;
		
	}
	
	/**
	 * Stacks up to two blocks onto each other using the claw,
	 * the LightSensor2 and the USS
	 * 
	 * @see ClawController
	 * @see LightPoller
	 * @see USPoller
	 */
	public void stack()
	{
		
	}
}
