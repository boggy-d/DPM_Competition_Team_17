/**
 * The TimeRecorder class calculates the time remaining
 * to the heat
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 * @since 0.1.0
 */

package component;

import lejos.utility.TimerListener;

public class TimeRecorder implements TimerListener{

	/**
	 * Records the time at the beginning of the competition
	 */
	public void setBeginningTime()
	{
		// TODO Figure out how to get the time.
	}
	
	/**
	 * Records the current time
	 */
	public void setCurrentTime()
	{
		// TODO Figure out how to get the time.
	}
	
	/**
	 * Returns the time remaining to the heat
	 */
	public void calculateTimeRemaining()
	{
		//TODO Simple subtraction to calculate time remaining
	}

	@Override
	public void timedOut() {
		// TODO Get data. Calculate time (basically use the above methods)
		
	}
}
