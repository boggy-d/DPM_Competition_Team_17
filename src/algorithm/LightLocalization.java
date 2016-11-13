/**
 * The LightLocalization class uses the LightSensor1
 * to travel to the assigned "origin" (i.e. one of the corners)
 * on the competition's grid field
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 */

package algorithm;

import component.ActionController;
import component.Odometer;
import component.Navigation;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalization {
	//for the constructor
	private Odometer odometer;
	private Navigation navigation;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	//motor speeds can be accessed in navigation
	private final int FORWARD_SPEED = 100;
	private final int ROTATION_SPEED = 60;
	private final int COLOR_THRESHOLD = 000;
	
	//for doLocalization 
	private final double COLOR_DIST = 0000;		//distance between the center of rotation of robot and light sensor
						
	
	/**
	 * Class constructor specifying the different parameters this class
	 * needs to works
	 * 
	 * @param odo the Odometer object to use
	 * @param navigation the Navigation object to use
	 * @param colorSensor the SampleProvider to use
	 * @param colorData the sample data to use
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor of the robot
	 * 
	 * @see Odometer
	 * @see Navigation
	 */
	public LightLocalization(Odometer odo, Navigation navigation, SampleProvider colorSensor, float[] colorData) {
		
		this.odo = odometer;
		this.navigation = navigation;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		
		
		//motors are static and may be accessed by action controller -- not sure how to do this
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}
	
	/**
	 * Localizes the robot when it gets to the assumed (0 , 0) coordinate
	 * using the LightSensor1 to detect floor lines while doing a full circle
	 * 
	 * @see LightPoller
	 * @see Navigation
	 */
	public void localize() {
		//array for the position of the robot
		double [] pos = new double [3];
		pos[2] = 0;															//set angle to 0, we won't be changing it
		
		//array for setPosition method (it will only update the X and Y position)
		boolean[] update = {true, true, false};
		
		//turn to approximate direction of lines
		navigation.turnTo(45);
	
		
		//setting speed to forward speed
		ActionController.setSpeeds(FORWARD_SPEED,FORWARD_SPEED, true);
		
		// drive forward until we detect a line
		while(getColorData() > 350)
		{ ActionController.stopMotors(); }
		
		 Sound.beep(); 
		
		
		
		//move backward until we are in the negative XY quadrant
		ActionController.goForward(COLOR_DIST + BUFFER_DIST, -FORWARD_SPEED);
		
		//stop motor
		ActionController.stopMotors();
		
		//line count
		int count = 0;
		
		//angles = {angleX negative, angleY positive, angleX positive, angleY negative}
		double[] angles = new double[4] ;
		
		//set rotation speed
		ActionController.setSpeeds(FORWARD_SPEED, -FORWARD_SPEED, true);
		
		//we need to cross 4 lines
		while (count < 4){
			
			//rotate clockwise -- already rotating
			
			//when we see a line
			if(getColorData() < COLOR_THRESHOLD){
				
				//store the angle in array
				 angles[count] = odo.getAng();
				 //increment line count
				 count++;			
			}
			
		}
		
		//stop motor
		ActionController.stopMotors();
		
		//update positions
		//position x
		pos[0] = -COLOR_DIST*Math.cos(Math.toRadians(angles[0]-angles[2])/2);
		
		//position y
		pos[1] = -COLOR_DIST*Math.cos(Math.toRadians(angles[1]-angles[3])/2);
		
		//update the position
		odo.setPosition(pos, update);
		
		//travel to (0,0)
		navigation.travelTo(0,0);
		
		//turn to 0 degrees
		navigation.turnTo(0);
		
		// update the odometer position 
		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
					

	}

	private float getColorData() {
		
		//TODO Remove and replace by LightPoller object
		
		colorSensor.fetchSample(colorData, 0);
		float color = colorData[0]*1000;
		
		return color;
	}
}
