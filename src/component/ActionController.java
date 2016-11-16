/**
 * The ActionController class acts as a finite state
 * machine for the system as it controls the flow
 * of the routine.
 * 
 * @author Bogdan Dumitru, Eric Zimmermann
 * @version 0.2.0
 * 
 */

package component;

import algorithm.ClawController;
import algorithm.LightLocalizer;
import algorithm.Navigator;
import algorithm.ObstacleAvoider;
import algorithm.USLocalizer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;
import lejos.utility.TimerListener;

public class ActionController implements TimerListener {
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawLift;
	private EV3LargeRegulatedMotor clawClose;
	private EV3UltrasonicSensor frontUsSensor;
	private EV3UltrasonicSensor sideUsSensor;
	private EV3UltrasonicSensor lightSensor;
	private EV3UltrasonicSensor colorSensor;

	static Odometer odometer;
	Navigator navigator;
	
	public ActionController(
			EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor clawLiftMotor,
			EV3LargeRegulatedMotor clawCloseMotor,
			EV3UltrasonicSensor frontUsSensor,
			EV3UltrasonicSensor sideUsSensor,
			EV3UltrasonicSensor lightSensor,
			EV3UltrasonicSensor colorSensor
			)
	{
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.clawLift = clawLiftMotor;
		this.clawClose = clawCloseMotor;
		this.frontUsSensor = frontUsSensor;
		this.sideUsSensor = sideUsSensor;
		this.lightSensor = lightSensor;
		this.colorSensor = colorSensor;

		odometer = new Odometer(leftMotor, rightMotor, 30, true);
	}
	

	/**
	 * Turns the left and right motors at
	 * the specified speeds
	 * 
	 * @param lSpd	the speed of the left motor
	 * @param rSpd	the speed of the right motor
	 */
	
	/*PROBLEM: if set speeds causes motors to actually move, the function gives user less control.
	 * This results in unnecessary movement that will interfere with methods in Navigation
	 * The robot will start rotating before actual scaled rotation in passed.
	 * Make a go backward method that uses distance.
	 * Make a generalized go forward/backward that doesnt take in any distances
	 * Just make a general setSpeeds that doesn't take make the bot move immediately
	 */
	
	public static void setSpeeds(int lSpd, int rSpd, boolean move) {

		leftMotor.setSpeed(Math.abs(lSpd));
		rightMotor.setSpeed(Math.abs(rSpd));
		
		if(move){
			
		if (lSpd < 0)
			leftMotor.backward();
		else
			leftMotor.forward();
		if (rSpd < 0)
			rightMotor.backward();
		else
			rightMotor.forward();
		}
	}

	/**
	 * Halts the motors
	 */
	public static void stopMotors() {

		setSpeeds(0,0,true);

		leftMotor.stop();
		rightMotor.stop();
	}

	/**
	 * Makes the robots advance or back up by a distance at a
	 * specified speed
	 * 
	 * @param distance the distance (in cm) to travel
	 * @param speed the speed of the wheels
	 */
	public static void goForward(double distance, int speed) {

		setSpeeds(speed, speed, true);

		double initial_X = odometer.getX();
		double initial_Y = odometer.getY();
		
		while ((Math.sqrt(Math.pow(initial_X - odometer.getX(), 2))
				+ Math.sqrt(Math.pow(initial_Y - odometer.getY(), 2))) < distance) {
			// do nothing / keep moving
		}

		stopMotors();
	}

	/**
	 * Gets the competition information provided by WiFi
	 * and stores it into fields
	 * 
	 * @see WifiConnection
	 */
	public void setWifiInfo() {
		
		//TODO Figure out how to get the wifi info
	}

	/**
	 * Sets up the odometer thread
	 * 
	 * @see Odometer
	 */
	public void startOdometer() {
		//TODO Instantiate odometer. Start odometer thread
	}

	/**
	 * Calculates the remaining time and returns if there is
	 * enough time to continue the routine
	 * 
	 * @return	<code>true</code> if the time is almost up, otherwise returns <code>false</code>
	 */
	public boolean isTimeShort() {

		//TODO Figure out how to get the time. Write algorithm to calculate remaining time
		
		return false;
	}

	/**
	 * Reads the <code>BTN</code> and <code>CTN</code> and returns the
	 * robots job for the heat
	 * @return <code>true</code> if the robot is a builder or <code>false</code> if it is a collector
	 */
	public boolean jobAssigned() {
		//TODO Write simple if-else. Create fields
		
		return false;

	}

	/**
	 * Moves the robot to the starting corner
	 */
	public void goToStart() {
		//TODO Write algorithm to go back to starting position while avoiding blocks
	}

	@Override
	public void timedOut() {
		// TODO EVERYTHING!!!
		setWifiInfo();
		
		// just for testing, actually get the values from the wifi
		double greenAreaX = 60;
		double greenAreaY = 60;

		startOdometer();
		
		// set up pollers
		USPoller frontUSPoller = new USPoller(frontUsSensor);
		LightPoller floorPoller = new LightPoller(lightSensor);
		LightPoller colorPoller = new LightPoller(colorSensor);

		Navigator navigator = new Navigator(odometer, frontUSPoller);

		// localize
		USLocalizer usLocalizer = new USLocalizer(odometer, frontUSPoller, leftMotor, rightMotor);
		usLocalizer.usLocalize();
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, navigator, floorPoller, leftMotor, rightMotor);
		lightLocalizer.lightlocalize();
		
		ClawController claw = new ClawController(clawLift, clawClose);
		ObstacleAvoider avoider = new ObstacleAvoider();

		//start USPoller timelistener here?
				
		// how to do this constantly? thread?
		// constantly check if there is a block in front
		boolean hasBlock = false;
		if (frontUSPoller.isBlock()) {
			ActionController.stopMotors();
			//possibly slow down and approach block
			
			if (colorPoller.isBlue()) {
				// pick up blue block
				claw.pickUpBlock();
				hasBlock = true;
			} else {
				//avoid obstacle
				avoider.avoidObstacle();
			}
		}
		
		// travel to middle of green zone
		navigator.travelTo(greenAreaX, greenAreaY);

		// do a 360 deg scan of the blocks around you
		double startingAngle = odometer.getAng();
		
		// start rotating clockwise
		ActionController.setSpeeds(Constants.ROTATION_SPEED, -Constants.ROTATION_SPEED, true);
		
		// avoid getting into while right away
        Delay.msDelay(1000);
		
		while (odometer.getAng() != startingAngle) {
			// store angles of the blocks in an array
			// sort blocks by the closest distance to you
		}
		// stop motors
		ActionController.stopMotors();
		
		// calculate the position of the blocks from the array of angles and distances
		// store in array
		
		// if you ran into a blue block on the way place it down
		if (hasBlock) {
			// TODO block placement algorithm
			claw.placeBlock(false);
			hasBlock = false;
		}
		
		// navigate to the closest block
		// remove that block position from the array
		
		// check if blue block or not
		
		// if it is a wooden block find the next closest block to you (avoid block if needed)
		// if there is no block (in the case that the opponent took it) find the next closet block to you and navigate to it

		// if it is a blue block pick it up
		
		// navigate back to the zone
		// place block where you want it
		// resort the positions of the blocks to the closest to you again
		// make sure you avoid the areas where the previous blocks were placed in the zone (or just avoid the zone)
		// navigate to that block and repeat
		
		// once you have finished checking all the blocks in the array
		// move to a distance 2x what the first scan could see from the zone
		// do a 360 scan and repeat
		
		
		
		
		
		// place block down
		claw.placeBlock(false);
		
	}
}
