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
		lightSampler = lightSensor.getMode("Red");
		lightData = new float[lightSampler.sampleSize()];
		
		this.colorSensor = colorSensor;
		colorSampler = colorSensor.getMode("RGB");	
		colorData = new float[colorSampler.sampleSize()];
		
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
		medianFilter.fetchSample(lightData, 0);
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
		//TODO Poll and parse LightSensor2 data
		lightSensor.fetchSample(colorData, 0);
		return colorData;		
	}
	
	/**
	 * Compares the value detected by LightSensor1
	 * to a threshold (i.e. if there's a line) and
	 * returns the result
	 * @return <code>true</code> if the light value is smaller than the threshold, otherwise returns <code>false</code>
	 */
	public boolean isLine(double prevLightData) {
		//TODO comparison filter
		//return getLightData() < Constants.BLACKINTENSITY;
		
		//get the current data
		double currentLightData = getLightData();
		
		//Passed a black line (compare the previous and current light data)
		//if the their difference is bigger than a threshold, we have passed a line
		if( prevLightData - currentLightData > Constants.LINE_DETECT_DIFF){
			return true;
		}
		
		else{
			return false;
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
		//TODO Implement Filters
		//return getColorData() == Constants.BLUECOLOURID;
		
		float[] colorData = getColorData();
		
		//block is blue (blue > red)
		if(colorData[2]*1000 > colorData[0]*1000){
			return true;
		}
		
		return false;	
	}

	@Override
	public void timedOut() {
		
		
		
		//TODO Get data. Check thresholds (basically use the above methods)
		
	}
}
