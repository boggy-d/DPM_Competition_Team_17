/**
 * The LCDInfo class gets the position and angle from the Odometer
 * class , the block status (block, obstacle, or none) from the 
 * LightPoller class, and the distance from the USPoller class and
 * prints them on the LCD screen of the EV3
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 */

package userInterface;

import lejos.utility.TimerListener;

public class LCDInfo implements TimerListener{

	/**
	 * Prints the position and heading of the robot, the
	 * block status and the distance to an object
	 * 
	 * @see Odometer
	 * @see LightPoller
	 * @see USPoller
	 */
	public void printDisplay()
	{
		//TODO Draw String
		
	}

	@Override
	public void timedOut() {
		// TODO Create Odometer, LightPoller, USPoller. Get info. 
		
	}
	
}
