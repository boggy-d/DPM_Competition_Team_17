package component;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.TimerListener;

public class USPoller implements TimerListener{
	private EV3UltrasonicSensor usSensor;
	private SampleProvider distanceSampler;
	private float[] usData;
	
	/**
	 * Class constructor specifying the USS sample feed and sample
	 * 
	 * @param usSensor the USS sample feed
	 * @param usData   the reading of the USS
	 */
	public USPoller(EV3UltrasonicSensor usSensor){
		this.usSensor = usSensor;
		distanceSampler = usSensor.getMode("Distance");
		usData = new float[distanceSampler.sampleSize()];
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
		return (getClippedData(255) < 5);

		
	}
	

	@Override
	public void timedOut() {
		//TODO Get data. Check thresholds (basically use the above methods)
		while(true){
			if (isBlock()) {
				
			}
		}
	}
}
