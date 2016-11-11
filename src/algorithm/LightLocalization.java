/**
 * The LightLocalization class uses the LightSensor1
 * to travel to the assigned "origin" (i.e. one of the corners)
 * on the competition's grid field
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 */

package algorithm;

import component.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalization {
	//for the constructor
	private Odometer odo;
	private Navigation navigation;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	//motor speeds
	private final int FORWARD_SPEED = 100;
	private final int ROTATION_SPEED = 60;
	
	//for doLocalization 
	private final double COLOR_DIST = 13.7;									//distance between the center of rotation of robot and light sensor
	private final double BUFFER_DIST = 6/Math.sqrt(2);						//buffer distance (where we want to be before rotating)
	
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
	public LightLocalization(Odometer odo, Navigation navigation, SampleProvider colorSensor, float[] colorData, 
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.navigation = navigation;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
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
	
		//save starting position (before light localization)
		double posXo = odo.getX();
		double posYo = odo.getY();
		
		//setting speed to forward speed
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		// drive forward until we detect a line
		while(getColorData() > 350){
			leftMotor.forward();
			rightMotor.forward();
		}
		
		 Sound.beep(); 
		
		/*
		 * Stop motor.
		 * We need to set speed to 0 before stopping the motor 
		 * because stop() doesn't stop both motors at the same time
		 * and leads to error (testing hardware issue).
		 */
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.stop();
		rightMotor.stop();
		
		//setting speed to forward speed
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		//save position we are at after having moved forward and seen a line
		double posX = odo.getX();
		double posY = odo.getY();
	
		//move backward until we are in the negative XY quadrant
		// we want to end up at BUFFER_DIST behind the lines (to make sure test is done in the right spot)
		while(odo.getX() > ( posX  - ((COLOR_DIST/Math.sqrt(2)) + BUFFER_DIST )  ) && 
				odo.getY() > ( posY  - ((COLOR_DIST/Math.sqrt(2)) + BUFFER_DIST)))
		{
			leftMotor.backward();
			rightMotor.backward();
		}
		
		//stop motor
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.stop();
		rightMotor.stop();
		
		//line count
		int count = 0;
		
		//angles = {angleX negative, angleY positive, angleX positive, angleY negative}
		double[] angles = new double[4] ;
		
		//set rotation speed
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		
		//we need to cross 4 lines
		while (count < 4){
			
			//rotate clockwise
			leftMotor.forward();
			rightMotor.backward();
			
			//when we see a line
			if(getColorData() < 500){
				
				//store the angle in array
				 angles[count] = odo.getAng();
				 //increment line count
				 count++;
				 Sound.beep(); 				
			}
			
		}
		
		//stop motor
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.stop();
		rightMotor.stop();
		
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
		
		//TODO Remove and replace by USPoller object
		
		colorSensor.fetchSample(colorData, 0);
		float color = colorData[0]*1000;
		
		return color;
	}
}
