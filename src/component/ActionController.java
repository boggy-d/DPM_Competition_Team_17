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
import algorithm.Searcher;
import algorithm.USLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
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
	public static Searcher searcher;
	public static ObstacleAvoider avoider;

	Point[] zone;	
	static Point[] restrictedZone;	
	Point blockLocation;
	int maxTowerHeight;
	int towerHeight;

	private Timer acTimer;
//	private double movementCounter = 0; // hold how many incremental pieces have been covered
	
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
		
		odometer = new Odometer(30, true, 0, 0, 0);	

		usPoller = new USPoller(Constants.frontUsSensor, Constants.sideUsSensor, Constants.DEFAULT_TIMEOUT_PERIOD, true);

		lightPoller = new LightPoller(Constants.lightSensor, Constants.colorSensor, Constants.DEFAULT_TIMEOUT_PERIOD, true);
		
		LCDInfo lcd = new LCDInfo();

		navigator = new Navigator();

		claw = new ClawController();
		
//		avoider = new ObstacleAvoider();

//		// localize
//		USLocalizer usLocalizer = new USLocalizer();
//		usLocalizer.usLocalize();
//
//		LightLocalizer lightLocalizer = new LightLocalizer();
//		lightLocalizer.lightlocalize();
//
//		// travel to origin and face 0 degrees
//		ActionController.navigator.travelTo(0,0);
//		ActionController.navigator.turnTo(0);

//		// TEST
//		while (true) {
//			if(lightPoller.getColorData()[2] > lightPoller.getColorData()[0]) {
//				Sound.beep();
//				Delay.msDelay(1000);
//
//			}
//				
//		}
		
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

		if (ROLE == 0) {
			// tower builder get green zone
			zone = getZoneCorners(LGZx, LGZy, UGZx, UGZy);
			restrictedZone = getZoneCorners(LRZx, LRZy, URZx, URZy);
			// set tower height
			maxTowerHeight = 2;

		} else {
			// garbage collector get red zone
			zone = getZoneCorners(LRZx, LRZy, URZx, URZy);
			restrictedZone = getZoneCorners(LGZx, LGZy, UGZx, UGZy);
			// set tower height
			maxTowerHeight = 1;
		}

		searcher = new Searcher(zone, restrictedZone, maxTowerHeight);
		
//		// navigate to the first corner of the zone while avoiding obstacles
//		avoider.start();
//		navigator.travelTo(zone[0].getX(), zone[0].getY());
//		avoider.stop();

		// search for blocks
//		searcher.start();
		searcher.search();

		// once it is done searching go back to home
		goToStart();

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
//		// TEST CASE 1
//		// set zones
//		LGZy = 8;
//		LGZx = 0;
//
//		UGZy = 9;
//		UGZx = 2;
//
//		LRZy = 6;
//		LRZx = 2;
//
//		URZy = 8;
//		URZx = 3;
//
//		// set starting corner
//		SC = 4;
//
//		// tower builder
//		ROLE = 0;
		
		// TEST CASE 2
		// set zones
		LGZy = 1;
		LGZx = 1;
		
		UGZy = 2;
		UGZx = 2;
		
		LRZy = 6;
		LRZx = 2;
		
		URZy = 8;
		URZx = 3;
		
		// set starting corner
		SC = 1;

		// tower builder
		ROLE = 0;
				
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
	public static double convertTilesToCm(int numberOfTiles) {
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
	public static Point calculatePosition(double[] currentPosition, double distanceAway) {
		// using trig calculate difference in x and y
		double deltaX = (distanceAway * Math.cos(currentPosition[2]));
		double deltaY = (distanceAway * Math.sin(currentPosition[2]));

		// add the difference in x and y to the current position
		Point position = new Point((float) (currentPosition[0] + deltaX), (float) (currentPosition[1] + deltaY));
		return position;
	}

	/**
	 * @param position the Point of the coordinates of a position you want to check
	 * @return true if it is in the arena, not in the opponents zone, and not in the corners, false if it is out of bounds
	 */
	public static boolean inBounds(Point position) {
		// check that is is within the x and y coordinates of the arena
		if (position.x < -Constants.TILE_LENGTH || position.x > convertTilesToCm(11) 
				|| position.y < -Constants.TILE_LENGTH || position.y > convertTilesToCm(11)) {
			return false;
		}
		
		// check if it is not within the other team's zone
		if ((position.x > restrictedZone[0].x && position.x < restrictedZone[1].x) 
				|| (position.y > restrictedZone[0].y && position.y < restrictedZone[2].y)) {
			return false;
		}
		
		// check that it is not in corner 1
		if (position.x < -Constants.TILE_LENGTH || position.y < 0) {
			return false;
		}
		
		// check that it is not in corner 2
		if (position.x > ActionController.convertTilesToCm(10) || position.y < 0) {
			return false;
		}
		
		// check that it is not in corner 3
		if (position.x > ActionController.convertTilesToCm(10) || position.y > ActionController.convertTilesToCm(10)) {
			return false;
		}
		
		// check that it is not in corner 4
		if (position.x < -Constants.TILE_LENGTH || position.y > ActionController.convertTilesToCm(10)) {
			return false;
			
		// if it is not in any of the restricted zones return true
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
	 * Moves the robot to the starting corner
	 */
	public void goToStart() {		
		// avoid obstacles on the way
//		avoider.start();
		
		if (SC == 1) {
			// Go back to bottom left corner
			navigator.travelTo(-Constants.DISTANCE_IN_CORNER, -Constants.DISTANCE_IN_CORNER);
		} else if (SC == 2) {
			// Go back to bottom right corner
			navigator.travelTo(convertTilesToCm(10) + Constants.DISTANCE_IN_CORNER, -Constants.DISTANCE_IN_CORNER);
		} else if (SC == 3) {	
			// Go back to upper right corner
			navigator.travelTo(convertTilesToCm(10) + Constants.DISTANCE_IN_CORNER, convertTilesToCm(10) + Constants.DISTANCE_IN_CORNER);
		} else {
			// Go back to upper left corner
			navigator.travelTo(-Constants.DISTANCE_IN_CORNER, convertTilesToCm(10) + Constants.DISTANCE_IN_CORNER);
		}
		
//		// stop avoider
//		avoider.stop();

	}

	@Override
	/**
	 * Main routine of robot
	 */
	public void timedOut() {

//		//Time is still remaining, do routine
//		if(time still remaining)
//		{
//			//Travel to not done, keep going to POI
//			if(navigation not done)
//			{
//				//Object in front detected
//				if(usPoller.isFrontBlock())
//				{
//					setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, true); //Get closer slower to detect color better
//					
//					//Block is blue, pick it up
//					if(lightPoller.isBlue())
//					{
//						claw.pickUpBlock();
//					}
//					
//					//Obstacle avoidance
//					else
//					{
//						
//					}
//				}
//				
//				//Keep navigating to POI
//				else
//				{
//					
//				}
//			}
//			
//			//Travel done, either search or place block
//			else
//			{
//				//Claw has a block already, place block and continue (might need to retravel to POI)
//				if(claw.isBlockGrabbed())
//				{
//					//Detected an existing block where the block is supposed to be placed, stack
//					if(usPoller.isFrontBlock() && lightPoller.isBlue())
//					{
//						claw.placeBlock(true);
//					}
//					//No block where the block is supposed to be placed, place block on ground
//					else
//					{
//						claw.placeBlock(false);
//					}
//				}
//				//Do searching algorithm
//				else(search)
//				{
//					
//				}
//			}
//		}
//		
//		//Round almost over, stop everything and go to start
//		else
//		{
//			//Avoid any block
//			if(usPoller.isFrontBlock())
//			{
//				
//			}
//			//Move to starting position
//			else
//			{
//				goToStart();
//			}
//			
//		}

	}
	// Navigation complete, start searching
	// Start 360 scan
	// if 360 scan hasnt been completed
	// 		if object detected and still in range activate motors and restart timed
	// 		out
	// 		else keep turning until it hits 360 deg
	// else go to second corner
}
