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

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import algorithm.ClawController;
import algorithm.LightLocalizer;
import algorithm.Navigator;
import algorithm.ObstacleAvoider;
import algorithm.Searching;
import algorithm.USLocalizer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.geometry.Point;
import lejos.utility.Delay;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import userInterface.LCDInfo;
import wifi.WifiConnection;

public class ActionController implements TimerListener {
	
	//Instantiate more objects
	public static Odometer odometer;
	public static Navigator navigator;
	public static USPoller usPoller;
	public static LightPoller lightPoller;
	public static ClawController claw;

	private Timer acTimer;
	private double movementCounter = 0; // hold how many incremental pieces have been covered
	
	//Create competition variables
	static int SC, ROLE, LRZx, LRZy, URZx, URZy, LGZx, LGZy, UGZx, UGZy;
	
	/**
	 * Class constructor that does localization and sets up the thread loop
	 * @param INTERVAL the refresh rate of the Timer
	 * @param autostart the flag that start the Timer on startup or not
	 */
	public ActionController(int INTERVAL, boolean autostart)
	{
		// set wifi info for testing only
		setTestWifiInfo();
		//setWifiInfo();
		
		odometer = new Odometer(30, true, 0, 0, 90);	

		usPoller = new USPoller(Constants.frontUsSensor, Constants.sideUsSensor, Constants.DEFAULT_TIMEOUT_PERIOD, true);
		
		LCDInfo lcd = new LCDInfo();
		
		lightPoller = new LightPoller(Constants.lightSensor, Constants.colorSensor, Constants.DEFAULT_TIMEOUT_PERIOD, true);
		navigator = new Navigator();
		
		claw = new ClawController();
		
		//Tests
//		navigator.turnTo(30);
//		navigator.travelTo(30, 30);
//		navigator.travelTo(60, 30);
//		navigator.travelTo(10, 70);
//		navigator.travelTo(50, 50);
//		Button.waitForAnyPress();
//		navigator.travelTo(1, 1);
//		Button.waitForAnyPress();
//		navigator.travelTo(30, 30);
//		Button.waitForAnyPress();
//		navigator.travelTo(0, 0);
//		Button.waitForAnyPress();
		
		// localize
		USLocalizer usLocalizer = new USLocalizer();
		usLocalizer.usLocalize();

		LightLocalizer lightLocalizer = new LightLocalizer();
		lightLocalizer.lightlocalize();

        // travel to origin and face 0 degrees
		ActionController.navigator.travelTo(0,0);
		ActionController.navigator.turnTo(0);

		// update position to the actual corner it starts in
		double angle = odometer.getAng();
		double[] position;
		boolean[] update = {true, true, true};
		if (SC == 1) {
			// Initialize to bottom left corner
			position = new double[] {0, 0, angle};	
		} else if (SC == 2) {
			// Initialize to bottom right corner
			position = new double[] {convertTilesToCm(10), 0, angle + 90};	
		} else if (SC == 3) {	
			// Initialize to upper right corner
			position = new double[] {convertTilesToCm(10), convertTilesToCm(10), navigator.wrapAngle(angle + 180)};	
		} else {
			// Initialize to upper left corner
			position = new double[] {0, convertTilesToCm(10), navigator.wrapAngle(angle - 90)};	
		}
		ActionController.odometer.setPosition(position, update);		
	
		Point[] zone;	
		if (ROLE == 0) {
			// tower builder get green zone
			zone = getZoneCorners(LGZx, LGZy, UGZx, UGZy);
		} else {
			// garbage collector get red zone
			zone = getZoneCorners(LRZx, LRZy, URZx, URZy);
		}
		
		// search for blocks
		Searching searcher = new Searching(zone);
		searcher.search();
		
		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			this.acTimer = new Timer((INTERVAL <= 0) ? INTERVAL : Constants.DEFAULT_TIMEOUT_PERIOD, this);
			this.acTimer.start();
		} else
			this.acTimer = null;		

	}
	
	
	/**
	 * Stops the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void stop() {
		if (acTimer != null)
			acTimer.stop();
	}
	
	/**
	 * Starts the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void start() {
		if (acTimer != null)
			acTimer.start();
	}
	/**
	 * Turns the left and right motors at
	 * the specified speeds
	 * 
	 * @param lSpd	the speed of the left motor
	 * @param rSpd	the speed of the right motor
	 */
	
	public static void setSpeeds(int lSpd, int rSpd, boolean move) {

		Constants.leftMotor.setSpeed(Math.abs(lSpd));
		Constants.rightMotor.setSpeed(Math.abs(rSpd));
		
		if(move){
			
		if (lSpd < 0)
			Constants.leftMotor.backward();
		else
			Constants.leftMotor.forward();
		if (rSpd < 0)
			Constants.rightMotor.backward();
		else
			Constants.rightMotor.forward();
		}
	}

	/**
	 * Halts the motors
	 */
	public static void stopMotors() {

		setSpeeds(0,0,true);

		Constants.leftMotor.stop();
		Constants.rightMotor.stop();
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
	 * For testing only
	 * sets "fake" wifi info
	 */
	public void setTestWifiInfo() {
		// set zones
		LGZy = 9;
		LGZx = 2;
		
		UGZy = 10;
		UGZx = 3;
		
		LRZy = 6;
		LRZx = 2;
		
		URZy = 8;
		URZx = 3;
		
		// set starting corner
		SC = 4;
			
		// set role
//		// garbage collector
//		ROLE = 1;

		// tower builder
		ROLE = 0;
		
//		// set zones
//		LGZy = 2;
//		LGZx = 2;
//		
//		UGZy = 3;
//		UGZx = 3;
//		
//		LRZy = 6;
//		LRZx = 2;
//		
//		URZy = 8;
//		URZx = 3;
//		
//		// set starting corner
//		SC = 1;
//			
//		// set role
////		// garbage collector
////		ROLE = 1;
//
//		// tower builder
//		ROLE = 0;
//		
	}
		
		
	/**
	 * Gets the competition information provided by WiFi
	 * and stores it into fields
	 * 
	 * @see WifiConnection
	 */
	public void setWifiInfo() {
		
		//Tries to connect to wifi
		WifiConnection conn = null;
		try {
			System.out.println("Connecting...");
			conn = new WifiConnection(Constants.SERVER_IP, Constants.TEAM_NUMBER, true);
		} catch (IOException e) {
			System.out.println("Connection failed");
		}
		
		if (conn != null) {
			HashMap<String, Integer> t = conn.StartData; //Get competition data
			
			if (t == null) {
				System.out.println("Failed to read transmission");
			} else {
				LGZy = t.get("LGZy");
				LGZx = t.get("LGZx");
				
				UGZy = t.get("LGZy");
				UGZx = t.get("LGZx");
				
				LRZy = t.get("LRZy");
				LRZx = t.get("LRZx");
				
				URZy = t.get("URZy");
				URZx = t.get("URZx");
				
				if(t.get("CTN") == Constants.TEAM_NUMBER)
				{
					// garbage collector
					SC = t.get("CSC");
					ROLE = 1;
				}
				else
				{
					// tower builder
					SC = t.get("BSC");
					ROLE = 0;
				}
				
				System.out.println(LGZy + " " + LGZx + " " + UGZy + " " + UGZx + " " + LRZy + " " + LRZx + " " + URZy + " " + URZx + " " + SC + " " + ROLE);
			}
		}
	}
	
	/**
	 * Converts the number of tiles into cm
	 * @param integer number of tiles
	 * @return a double of the position in cm
	 */
	public double convertTilesToCm(int numberOfTiles) {
		return numberOfTiles * Constants.TILE_LENGTH;
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
	 * Calculates the position of something at a distance from the current position
	 * @param double array of the x coordinate, y coordinate, and the angle of the current position
	 * @param the distance of the object from the current position
	 * @return the position of the the object
	 */
	Point calculatePosition(double[] currentPosition, double distanceAway) {
		// using trig calculate difference in x and y
		double deltaX = (distanceAway * Math.cos(currentPosition[2]));
		double deltaY = (distanceAway * Math.sin(currentPosition[2]));
		
		// add the difference in x and y to the current position
		Point position = new Point((float) (currentPosition[0] + deltaX), (float) (currentPosition[1] + deltaY));
		return position;
	}
	
	/**
	 * @param the coordinates of a position
	 * @return true if it is in the arena, false if it is outside
	 */
	boolean inBounds(Point position){
		if (position.x < -Constants.TILE_LENGTH || position.x > convertTilesToCm(11) 
				|| position.y < -Constants.TILE_LENGTH || position.y > convertTilesToCm(11)) {
			return false;
		} else {
			return true;
		}
	}
	
	/**
	 * Given the lower left and upper right corners of a zone
	 * @param lowerLeftX
	 * @param lowerLeftY
	 * @param upperRightX
	 * @param upperRightY
	 * @return an array of points of all the corners of the zone
	 */
	public Point[] getZoneCorners(int lowerLeftX, int lowerLeftY, int upperRightX, int upperRightY) {
		Point[] corners = new Point[4];
		
		// convert from tiles to Cm
		double lowerLeftXCm = convertTilesToCm(lowerLeftX);
		double lowerLeftYCm = convertTilesToCm(lowerLeftY);
		double upperRightXCm = convertTilesToCm(upperRightX);
		double upperRightYCm = convertTilesToCm(upperRightY);

		// lower left
		Point lowerLeft = new Point((float)lowerLeftXCm, (float)lowerLeftYCm);
		Point lowerRight = new Point((float)upperRightXCm, (float)lowerLeftYCm);
		Point upperLeft = new Point((float)lowerLeftYCm, (float)upperRightYCm);
		Point upperRight = new Point((float)upperRightXCm, (float)upperRightYCm);

		corners[0] = lowerLeft;
		corners[1] = lowerRight;
		corners[2] = upperLeft;
		corners[3] = upperRight;

		return corners;
	}
	
	/**
	 * Approach the block and check if it is a wooden or foam block,
	 * if it is a wooden block go back and continue scanning for blocks
	 * if it is a blue block, pick it up and place or stack it
	 * @param endingAngle
	 * @param angleOfBlock
	 * @param cornerX
	 * @param cornerY
	 */
	public void getBlock(double endingAngle, double angleOfBlock, int cornerX, int cornerY) {
		boolean hasBlock = false;

		// go forward towards block
		ActionController.setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, true);
		
		// keep checking if there is a block ahead
		while (true) {
			// when there is a block ahead break out of the loop
			if (usPoller.isFrontBlock()) {
				break;
			}
		}
		
		// if it is a blue block pick it up
		if (lightPoller.isBlue()) {
			//Claw pickup routine
			claw.pickUpBlock();
			hasBlock = true;
		}
		
		// if it has a block place it
		if (hasBlock) {
			// TODO block placing algorithm
			
			// go back to the corner
			navigator.travelTo(cornerX, cornerY);
		} else {
			// go back to the corner
			navigator.travelTo(cornerX, cornerY);
			
			// if it doesn't have block continue scanning where it left off
			navigator.turnTo(angleOfBlock);
			
			// don't start scanning until you've passed the obstacle
			while (true) {
				// start rotating counter clockwise
				ActionController.setSpeeds(-Constants.ROTATION_SPEED, Constants.ROTATION_SPEED, true);
				
				double distance1 = usPoller.getClippedData(255);
				
				// delay for a bit
		        Delay.msDelay(Constants.DELAY_MS);
				
				double distance2 = usPoller.getClippedData(255);
				
				// check if the scan has passed the obstacle
				if ((distance2 - distance1) > Constants.DISTANCE_DIFFERENCE) {
					break;
				}
			}
		}
		//start scanning again
		scanForBlocks(endingAngle, cornerX, cornerY);
	}
	

	/**
	 * Moves the robot to the starting corner
	 */
	public void goToStart() {
		//TODO Write algorithm to go back to starting position while avoiding blocks
	}
	
	

	@Override
	/**
	 * Main routine of robot
	 */
	public void timedOut() {

		// Objected is detected
		if (usPoller.isFrontBlock()) {

			// Block is blue
			if (lightPoller.isBlue() && !claw.isBlockGrabbed()) {
				// Claw pickup routine
				claw.pickUpBlock();
				claw.setBlockGrabbed(true);
			}

			else if (!claw.isBlockGrabbed()) {
				claw.placeBlock(true);
				claw.setBlockGrabbed(false);
			}

			// Block is obstacle or block is already captured
			else {
				// Obstacle avoidance
			}
		}

		// Navigate
		else {
			if(movementCounter != Constants.MOVEMENT_PARTITIONS)
			{
				Constants.leftMotor.rotate((int) navigator.dR, true);
				Constants.rightMotor.rotate((int) navigator.dR, false);
			}
		}
	}


	// Navigation complete, start searching
	// Start 360 scan
	// if 360 scan hasnt been completed
	// 		if object detected and still in range activate motors and restart timed
	// 		out
	// 		else keep turning until it hits 360 deg
	// else go to second corner

}
