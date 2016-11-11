/**
 * The Collector class contains methods related to the competition
 * collector role i.e. navigating to the red zone, picking up blocks, and dropping off blocks
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 */

package algorithm;

public class Collector {
	
	/**
	 * Navigates to the wifi assigned Red Zone
	 * while avoiding blocks
	 * 
	 * @see ActionController
	 * @see Navigation
	 * @see ObstacleAvoidance
	 */
	public void goToRed()
	{
		
	}
	
	/**
	 * Uses the LightPoller to detect the color of the
	 * floor in addition to checking if the
	 * wifi coordinates match. Returns if it is red or not.
	 * 
	 * @return <code>true</code> if the robot reached a red zone, otherwise return <code>false</code>
	 * 
	 * @see ActionController
	 * @see LightPoller
	 */
	public boolean isRed()
	{
		return false;
		
	}
	
	/**
	 * Places up to n (TBD) blocks in the red zone using the claw,
	 * the LightSensor2 and the USS
	 * 
	 * @see ActionController
	 * @see LightPoller
	 * @see USPoller
	 */
	public void place()
	{
		
	}

}
