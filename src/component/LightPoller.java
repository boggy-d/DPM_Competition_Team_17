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

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.TimerListener;

public class LightPoller implements TimerListener{
	private EV3UltrasonicSensor lightSensor;
	private SampleProvider lightSampler;
	private float[] lightData;
	
	double BLACKINTENSITY = 0.2;

	// initializes color sensor.
	// method with variable filters for different needs
	// one filter for black gridlines
	// one filter for object detection
	
	/**
	 * Class Constructor
	 * 
	 * @since 0.1.0
	 */
	public LightPoller(EV3UltrasonicSensor lightSensor)
	{
		//TODO Modify constructor parameters. Create appropriate fields. Assigne params to fields
		this.lightSensor = lightSensor;
		lightSampler = lightSensor.getMode("Red");
		lightData = new float[lightSampler.sampleSize()];			// colorData is the buffer in which data are returned
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
		return lightData[0];
		
	}
	
	/**
	 * Polls the LightSensor2 and returns the
	 * an array containing the amount of light reflected
	 * of each color
	 * @return	an array containing the red, blue and green light values between 0 and 100, with 0 = low and 100 = high
	 */
	public double[] getColorData()
	{
		//TODO Poll and parse LightSensor2 data
		return null;
		
	}
	
	/**
	 * Compares the value detected by LightSensor1
	 * to a threshold (i.e. if there's a line) and
	 * returns the result
	 * @return <code>true</code> if the light value is smaller than the threshold, otherwise returns <code>false</code>
	 */
	public boolean isLine() {
		//TODO comparison filter
		return getLightData() < BLACKINTENSITY;
	}
	
	/**
	 * Compares the values detected by LightSensor2
	 * to thresholds (i.e. if the block is blue) and
	 * returns the result
	 * @return <code>true</code> if the blue light is bigger than the threshold, otherwise returns <code>false</code>
	 */
	public boolean isBlue()
	{
		//TODO Write algorithm for blue block detection. Implement Filters
		
		return false;
		
	}

	@Override
	public void timedOut() {
		//TODO Get data. Check thresholds (basically use the above methods)
		
	}
}
