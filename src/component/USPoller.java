/**
 * The USPoller uses the ultrasonic sensors to poll the distance between
 * them and the object in front and uses thresholds to determine if a block is close or
 * not
 */
package component;

import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class USPoller{
	private SensorModes frontSensor;
	private SampleProvider frontSampler;
	private float[] frontUsData, sideUsData;
	private Timer usPollerTimer;
	private boolean isFrontBlock, isSideBlock;
	private SensorModes sideSensor;
	private SensorMode sideSampler;

	/**
	 * Initializes the ultrasonic sensors
	 * @param frontSensor the front ultrasonic sensor
	 * @param sideSensor the side ultratsonic sensor
	 */
	public USPoller(SensorModes frontSensor, SensorModes sideSensor){
		this.frontSensor = frontSensor;
		frontSampler = frontSensor.getMode("Distance");
		
		this.sideSensor = sideSensor;
		sideSampler = sideSensor.getMode("Distance");
		
		this.frontUsData = new float[frontSampler.sampleSize()];
		this.sideUsData = new float[sideSampler.sampleSize()];

		
	}

	/**
	 * Returns the distance reported by the USS,
	 * clipping it to <code>maxValue</code> if it is
	 * too high
	 * @param maxValue the cutoff distance
	 * @return         the distance reported by the USS times a factor (for visibility)
	 */
	public float getClippedData(SensorModes usSensor, float[] usData, int maxValue){
		
		usSensor.fetchSample(usData,0);
		float distance = usData[0]*100;

		if(distance > maxValue)
		{ distance = maxValue; }
		
		return distance;
	}
	
	/**
	 * Returns the distance reported by the USS
	 *
	 * @return	the distance reported by the USS times a factor (for visibility)
	 */
	
	/**
	 * Returns the distance reported by the USS
	 * @param usSensor the ultrasonic sensor
	 * @param usData the data 
	 * @return the distance reported by the USS times a factor (for visibility)
	 */
	public float getRawData(SensorModes usSensor, float[] usData) {
		
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;
			 
		return distance;
	}
	
	/**
	 * Compares the value detected by the front USS
	 * to a threshold (i.e. if there's an obstacle) and
	 * returns the result
	 * @return <code>true</code> if the distance is smaller than the threshold, otherwise returns <code>false</code>
	 */
	public boolean isFrontBlock()
	{
		if(getFrontDistance(Constants.SEARCHING_CLIP) <= Constants.BLOCK_INFRONT)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	/**
	 * Compares the value detected by the side USS
	 * to a threshold (i.e. if there's an obstacle) and
	 * returns the result
	 * @return true if the side USS detects a block, false otherwise
	 */
	public boolean isSideBlock()
	{
		if(getSideDistance() < Constants.BLOCK_INFRONT)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	/**
	 * Returns the distance reported by the front USS wrt the
	 * maximum value to clip
	 * @param maxValue the maximum value allowed by the USS
	 * @return the distance detected by the USS
	 */
	public double getFrontDistance(int maxValue)
	{
		{
			return getClippedData(frontSensor, frontUsData, maxValue);

		}
	}
	
	/**
	 * Polls the side USS and returns the distance it detected
	 * @return the distance detected by the USS
	 */
	public double getSideDistance()
	{
		{
			return getRawData(sideSensor, sideUsData);
		}
	}

}
