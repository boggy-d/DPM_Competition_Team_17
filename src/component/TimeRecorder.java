/**
 * The TimeRecorder class calculates the time remaining
 * to the heat
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 * @since 0.1.0
 */

package component;

import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class TimeRecorder{
	
	private long beginningTime, currentTime, timeRemaining;
	private boolean isTimeRemaining;
	private Timer timeTimer;
	
	public TimeRecorder()
	{
		setBeginningTime();
//		if (autostart) {
//			// if the timeout interval is given as <= 0, default to 20ms timeout 
//			timeTimer = new Timer((INTERVAL <= 0) ? INTERVAL : Constants.DEFAULT_TIMEOUT_PERIOD, this);
//			timeTimer.start();
//		} else
//			timeTimer = null;
	}
	
	/**
	 * Stops the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void stop() {
		if (timeTimer != null)
			timeTimer.stop();
	}
	
	/**
	 * Starts the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void start() {
		if (timeTimer != null)
			timeTimer.start();
	}
	
	/**
	 * Records the time at the beginning of the competition
	 */
	public void setBeginningTime()
	{
		// TODO Figure out how to get the time.
		this.beginningTime = System.currentTimeMillis();
	}
	
	/**
	 * Records the current time
	 */
	public void setCurrentTime()
	{
		// TODO Figure out how to get the time.
		this.currentTime = System.currentTimeMillis();
	}
	
	/**
	 * Returns the time remaining to the heat
	 */
	public long calculateTimeRemaining()
	{
		//TODO Simple subtraction to calculate time remaining
		return (Constants.FIVE_MINUTES + this.beginningTime) - this.currentTime;
	}
	
	public boolean isTimeRemaining()
	{
		setCurrentTime();

		if (calculateTimeRemaining() < Constants.TIME_TO_GO_TO_START) {
			isTimeRemaining = false;
		} else {
			isTimeRemaining = true;
		}
		return isTimeRemaining;
	}

//	@Override
//	public void timedOut() {
//		// TODO Get data. Calculate time (basically use the above methods)
//		setCurrentTime();
//
//		if (calculateTimeRemaining() < Constants.TIME_TO_GO_TO_START) {
//			isTimeRemaining = false;
//		} else {
//			isTimeRemaining = true;
//		}
//		
//		
//	}
}
