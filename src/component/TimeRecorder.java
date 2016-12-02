/**
 * The TimeRecorder class calculates the time remaining
 * to the heat
 * 
 * @author Bogdan Dumitru
 */

package component;

import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class TimeRecorder{
	
	private long beginningTime, currentTime, timeRemaining;
	private boolean isTimeRemaining;
	private Timer timeTimer;
	
	/**
	 * Sets the beginning time of the competition
	 */
	public TimeRecorder()
	{
		setBeginningTime();
	}
	
	/**
	 * Records the time at the beginning of the competition
	 */
	public void setBeginningTime()
	{
		this.beginningTime = System.currentTimeMillis();
	}
	
	/**
	 * Records the current time
	 */
	public void setCurrentTime()
	{
		this.currentTime = System.currentTimeMillis();
	}
	
	/**
	 * Returns the time remaining to the round
	 */
	public long calculateTimeRemaining()
	{
		return (Constants.FIVE_MINUTES + this.beginningTime) - this.currentTime;
	}
	
	/**
	 * Evaluates the time remaining of the round to 
	 * see if the robot needs to return to the starting
	 * corner or not
	 * @return true if there is time remaining before returning to the starting corner, false otherwise
	 */
	public boolean isTimeRemaining()
	{
		setCurrentTime();

		if (calculateTimeRemaining() < Constants.TIME_TO_GO_TO_START) {
			isTimeRemaining = false;

			ActionController.goToStart();
		}
		else
		{
			isTimeRemaining = true;
		}
		return isTimeRemaining;
	}
}
