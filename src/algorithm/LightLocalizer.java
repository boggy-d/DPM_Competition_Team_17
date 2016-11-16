
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
import component.LightPoller;
import component.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	//for the constructor
	private Odometer odometer;
	private Navigator navigator;
	private LightPoller lightPoller;
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
	 * @param odometer the Odometer object to use
	 * @param navigation the Navigation object to use
	 * @param colorSensor the EV3UltrasonicSensor to use
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor of the robot
	 *
	 * @see Odometer
	 * @see Navigator
	 */
	public LightLocalizer(Odometer odometer, Navigator navigator, LightPoller lightPoller,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.navigator = navigator;
		this.lightPoller = lightPoller;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	/**
	 * Localizes the robot when it gets to the assumed (0 , 0) coordinate
	 * using the LightSensor1 to detect floor lines while doing a full circle
	 * 
	 * @see LightPoller
	 * @see Navigator
	 */
	public void lightlocalize() {
	
		//turn to approximate direction of lines
		navigator.turnTo(45);
		
		//setting speed and begin to move forward speed
		ActionController.setSpeeds(FORWARD_SPEED,FORWARD_SPEED, true);
		
		// drive forward until we detect a line
		while (!lightPoller.isLine()) {
		    //keep moving forward
		}
		
		// black line detected
		ActionController.stopMotors();
		Sound.beep();
		
		
		// move backward until we are in the negative XY quadrant
		ActionController.goForward((float)(COLOR_DIST + BUFFER_DIST), -FORWARD_SPEED);
		
		// start rotating clockwise
		ActionController.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED, true);

       		 // count lines crossed and store the orientation at each line
       		 int lineCount = 0; 
		
		//angles = {angleX negative, angleY positive, angleX positive, angleY negative}
		double[] angles = new double[4] ;
		
		while (lineCount < 4) {
			if (lightPoller.isLine()) {
				// store angle in array
				angles[lineCount++] = odometer.getAng();
				// play sound to confirm
				Sound.beep();
				// avoid counting same line several times
                Delay.msDelay(2000);
			}
		}

		// stop motors
		ActionController.stopMotors();
        

        // calculate correct x, y and theta differences
        // using formulas from the tutorial slides 
        double negativeYTheta = angles[3];
        double deltaXTheta = angles[0] - angles[2];
        double deltaYTheta = angles[1] - angles[3];		// or 3-1? check

        double deltaX = -COLOR_DIST * Math.cos(Math.toRadians(deltaYTheta) / 2);
        double deltaY = -COLOR_DIST * Math.cos(Math.toRadians(deltaXTheta) / 2);
        double deltaTheta = 180 - negativeYTheta + deltaYTheta / 2;		//check math

        //TODO check if it is supposed to be added to or not
        // correct coordinates and orientation
		double[] correction = {odometer.getX() + deltaX, odometer.getY() + deltaY, odometer.getAng() + deltaTheta};
		boolean[] update = {true, true, true};
		odometer.setPosition(correction, update);

	//put this outside lightlocalizer?
        // travel to origin and face 0 degrees
		navigator.travelTo(0,0);
		navigator.turnTo(0);
		
		//beep when finished localizing
	}
}

