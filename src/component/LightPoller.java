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
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.robotics.filter.MedianFilter;

public class LightPoller {
	private SensorModes lightSensor, colorSensor;
	private SampleProvider lightSampler, colorSampler;
	private float[] lightData, colorData;
	private MedianFilter medianFilter;
	private Timer lightPollerTimer;
	private double ambientLight;
	private boolean isLine, isBlue;
	
	/**
	 * Class Constructor that initializes the sensors
	 */

	public LightPoller(SensorModes lightSensor, SensorModes colorSensor)
	{
		
		this.lightSensor = lightSensor;
		this.lightSampler = this.lightSensor.getMode("Red");
		this.lightSensor.setCurrentMode("Red");
		this.lightData = new float[lightSampler.sampleSize()];
		
		this.colorSensor = colorSensor;
		this.colorSampler = this.colorSensor.getMode("RGB");
		this.colorData = new float[colorSampler.sampleSize()];
		
		lightSampler.fetchSample(lightData, 0);
		
		//calibrate ambient light
		for (int i = 0; i < 10; ++i)
		{ambientLight += lightData[0]; }
		
		ambientLight = ambientLight / 10;
		
	}
	
	/**
	 * Polls the LightSensor1 and returns the
	 * amount of light the LightSensor1 detected 
	 * @return the light value between 0 and 100, with 0 = light and 100 = dark
	 */
	public double getLightData()
	{
		
		lightSensor.fetchSample(lightData, 0);
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
		if(Math.abs(ambientLight - getLightData()) >= Constants.LINE_DETECT_DIFF)
		{
			return true;
		}
		else
		{
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
		if(getColorData()[2] > (getColorData()[0]))
		{
			return true;
		}
		else
		{
			return false;
		}

	}

}
