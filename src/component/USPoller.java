package component;

import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class USPoller implements TimerListener{
	private SensorModes usSensor;
	private SampleProvider distanceSampler;
	private float[] usData;
	private Timer usPollerTimer;
	private boolean isBlock;
	private Object lock;
	
	/**
	 * Class constructor specifying the USS sample feed and sample
	 * 
	 * @param usSensor the USS sample feed
	 * @param usData   the reading of the USS
	 */
	public USPoller(SensorModes usSensor, int INTERVAL, boolean autostart){
		this.usSensor = usSensor;
		distanceSampler = usSensor.getMode("Distance");
		usData = new float[distanceSampler.sampleSize()];
		
		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			usPollerTimer = new Timer((INTERVAL <= 0) ? INTERVAL : Constants.DEFAULT_TIMEOUT_PERIOD, this);
			usPollerTimer.start();
		} else
			usPollerTimer = null;
		
	}
	
	/**
	 * Stops the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void stop() {
		if (usPollerTimer != null)
			usPollerTimer.stop();
	}
	
	/**
	 * Starts the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void start() {
		if (usPollerTimer != null)
			usPollerTimer.start();
	}

	/**
	 * Returns the distance reported by the USS,
	 * clipping it to <code>maxValue</code> if it is
	 * too high
	 * @param maxValue the cutoff distance
	 * @return         the distance reported by the USS times a factor (for visibility)
	 */
	public float getClippedData(int maxValue){
		
		usSensor.fetchSample(usData,0);
		float distance = usData[0]*100;

		if(distance > maxValue)
		{ distance = maxValue; }
		
		return distance;
	}
	
	/**
	 * Returns the distance reported by the USS
	 * @return	the distance reported by the USS times a factor (for visibility)
	 */
	public float getRawData() {

		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;
			 
		return distance;
	}
	
	/**
	 * Compares the value detected by the USS
	 * to a threshold (i.e. if there's an obstacle) and
	 * returns the result
	 * @return <code>true</code> if the distance is smaller than the threshold, otherwise returns <code>false</code>
	 */
	public boolean isBlock()
	{
		//TODO Write algorithm for block detection. Implement filters.
		synchronized(lock)
		{
			return isBlock;
		}	
	}
	

	@Override
	public void timedOut() {
		//TODO Get data. Check thresholds (basically use the above methods)
		//TODO add constants
		if(getClippedData(255) < 5)
		{
			isBlock = true;
		}
		else
		{
			isBlock = false;
		}
	}
}
