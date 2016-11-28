package component;

import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class USPoller implements TimerListener{
	private SensorModes frontSensor;
	private SampleProvider frontSampler;
	private float[] usData;
	private Timer usPollerTimer;
	private boolean isFrontBlock, isSideBlock;
	private SensorModes sideSensor;
	private SensorMode sideSampler;

	/**
	 * Class constructor specifying the USS sample feed and sample
	 * @param frontSensor
	 * @param sideSensor
	 * @param INTERVAL
	 * @param autostart
	 */
	public USPoller(SensorModes frontSensor, SensorModes sideSensor, int INTERVAL, boolean autostart){
		this.frontSensor = frontSensor;
		frontSampler = frontSensor.getMode("Distance");
		
		this.sideSensor = sideSensor;
		sideSampler = sideSensor.getMode("Distance");
		
		this.usData = new float[frontSampler.sampleSize()];
		
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
	public float getClippedData(SensorModes usSensor, int maxValue){
		
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
	public float getRawData(SensorModes usSensor) {
		
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
	public boolean isFrontBlock()
	{
		//TODO Write algorithm for block detection. Implement filters.
		synchronized(this)
		{
			return isFrontBlock;
		}	
	}
	
	public boolean isSideBlock()
	{
		//TODO Write algorithm for block detection. Implement filters.
		synchronized(this)
		{
			return isSideBlock;
		}	
	}
	
	public double getFrontDistance()
	{
		synchronized(this)
		{
			return getRawData(frontSensor);
		}
	}
	
	public double getSideDistance()
	{
		synchronized(this)
		{
			return getRawData(sideSensor);
		}
	}

	@Override
	/**
	 * Constantly checks if the front and side USSS detect a block
	 */
	public void timedOut() {
		//TODO Get data. Check thresholds (basically use the above methods)
		//TODO add constants
		if(getClippedData(frontSensor,Constants.CLIP) < Constants.BLOCK_INFRONT)
		{
			isFrontBlock = true;
		}
		else
		{
			isFrontBlock = false;
		}
		
		if(getClippedData(sideSensor, Constants.CLIP) < Constants.BLOCK_INFRONT)
		{
			isSideBlock = true;
		}
		else
		{
			isSideBlock = false;
		}
	}
}
