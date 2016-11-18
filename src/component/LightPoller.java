/**
 * The LightPoller class polls and parses both light sensors in
 * order to detect lines and blocks.
 * The sensor in Red mode will be defined as LightSensor1.
 * The sensor in Color mode will be defined as LightSensor2.
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 * @since 0.1.0
 */

package component;

import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.robotics.filter.MedianFilter;

public class LightPoller implements TimerListener{
	private SensorModes lightSensor, colorSensor;
	private SampleProvider lightSampler, colorSampler;
	private float[] lightData, colorData;
	private MedianFilter medianFilter;
	private Timer lightPollerTimer;
	private double ambientLight;
	private boolean isLine, isBlue;
	private Object lock;

	// initializes color sensor.
	// method with variable filters for different needs
	// one filter for black gridlines
	// one filter for object detection
	
	/**
	 * Class Constructor
	 * 
	 * @since 0.1.0
	 */
	public LightPoller(SensorModes lightSensor, SensorModes colorSensor, int INTERVAL, boolean autostart)
	{
		//TODO Modify constructor parameters. Create appropriate fields. Assigne params to fields
		this.lightSensor = lightSensor;
		this.lightSampler = this.lightSensor.getMode("Red");
		this.lightData = new float[lightSampler.sampleSize()];
		
		this.colorSensor = colorSensor;
		this.colorSampler = this.colorSensor.getMode("RGB");	
		this.colorData = new float[colorSampler.sampleSize()];
		
		lightSampler.fetchSample(lightData, 0);
		ambientLight = lightData[0];
		
		this.medianFilter = new MedianFilter(lightSampler, 5);
		
		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			lightPollerTimer = new Timer((INTERVAL <= 0) ? INTERVAL : Constants.DEFAULT_TIMEOUT_PERIOD, this);
			lightPollerTimer.start();
		} else
			lightPollerTimer = null;
		
	}
	
	/**
	 * Stops the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void stop() {
		if (lightPollerTimer != null)
			lightPollerTimer.stop();
	}
	
	/**
	 * Starts the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void start() {
		if (lightPollerTimer != null)
			lightPollerTimer.start();
	}
	
	/**
	 * Polls the LightSensor1 and returns the
	 * amount of light the LightSensor1 detected 
	 * @return the light value between 0 and 100, with 0 = light and 100 = dark
	 */
	public double getLightData()
	{
		//TODO filter
		
		lightSensor.fetchSample(lightData, 0);
//		medianFilter.fetchSample(lightData, 0);
		return lightData[0];
		
	}
	
	/**
	 * Polls the LightSensor2 and returns the
	 * an array containing the amount of light reflected
	 * of each color
	 * @return	an array containing the red, blue and green light values between 0 and 100, with 0 = low and 100 = high
	 */
	public float[] getColorData()
	{
		colorSensor.fetchSample(colorData, 0);
		return colorData;		
	}
	
	/**
	 * Compares the value detected by LightSensor1
	 * to a threshold (i.e. if there's a line) and
	 * returns the result
	 * @return <code>true</code> if the light value is smaller than the threshold, otherwise returns <code>false</code>
	 */
	public boolean isLine() {
		synchronized(lock)
		{
			return isLine;
		}
	}
	
	/**
	 * Compares the values detected by LightSensor2
	 * to thresholds (i.e. if the block is blue) and
	 * returns the result
	 * @return <code>true</code> if the blue light is bigger than the threshold, otherwise returns <code>false</code>
	 */
	public boolean isBlue()
	{
		synchronized(lock)
		{
			return isBlue;
		}
	}

	@Override
	public void timedOut() {
		//TODO Implement Filters
		if(Math.abs(ambientLight - getLightData()) >= Constants.LINE_DETECT_DIFF)
		{
			isLine = true;
		}
		else
		{
			isLine = false;
		}
		
		if(getColorData()[2] > getColorData()[0])
		{
			isBlue = true;
		}
		else
		{
			isBlue = false;
		}
		
	}
}
