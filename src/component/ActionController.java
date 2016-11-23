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
	public static USPoller frontUsPoller;
	public static USPoller sideUSPoller;
	public static LightPoller lightPoller;
	public static ClawController claw;

	private Timer acTimer;
	
	//Create competition variables
	static int SC, ROLE, LRZx, LRZy, URZx, URZy, LGZx, LGZy, UGZx, UGZy;
	
	public ActionController(int INTERVAL, boolean autostart)
	{
		// set wifi info for testing only
		setTestWifiInfo();
		
//		//Wifi "supposedly works (router is bad and it should feel bad)
//		setWifiInfo();
		
		odometer = new Odometer(30, true, 0, 0, 90);	

		frontUsPoller = new USPoller(Constants.frontUsSensor, /* sideUsSensor, */ Constants.DEFAULT_TIMEOUT_PERIOD, true);
		
		sideUSPoller = new USPoller(Constants.sideUsSensor, Constants.DEFAULT_TIMEOUT_PERIOD, true);
		
		LCDInfo lcd = new LCDInfo();
		
		lightPoller = new LightPoller(Constants.lightSensor, Constants.colorSensor, Constants.DEFAULT_TIMEOUT_PERIOD, true);
	
		navigator = new Navigator();
		
		claw = new ClawController();
		
		//Tests
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
		search(zone);
		
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
	
	/*PROBLEM: if set speeds causes motors to actually move, the function gives user less control.
	 * This results in unnecessary movement that will interfere with methods in Navigation
	 * The robot will start rotating before actual scaled rotation in passed.
	 * Make a go backward method that uses distance.
	 * Make a generalized go forward/backward that doesnt take in any distances
	 * Just make a general setSpeeds that doesn't take make the bot move immediately
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
	 * @param position, the Point of the coordinates of a position
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
	 * Scan for blocks
	 */
	public void scanForBlocks(double endingAngle, int cornerX, int cornerY) {		
		// scan until it reaches ending angle
		while (odometer.getAng() < endingAngle) {
			// start rotating counter clockwise
			ActionController.setSpeeds(-Constants.ROTATION_SPEED, Constants.ROTATION_SPEED, true);
			
			double distance = frontUsPoller.getClippedData(255);
			Point blockPosition = calculatePosition(odometer.getPosition(), distance);
			
			// if the distance is less than the distance the sensor can see and the block is not out of bounds (a wall)
			if (distance < Constants.SEARCH_DISTANCE_THRESHOLD & inBounds(blockPosition)) {
				// test this delay to see what is a good time
				// delay to make it face the block, not just the edge
				Delay.msDelay(500);
				
				// once it sees a block stop
				ActionController.stopMotors();
				
				// is is a block, check what block it is 
				getBlock(endingAngle, odometer.getAng(), cornerX, cornerY);
				return;
			}
		}
		// no block found, return 
		return;
	}
	
	/**
	 * Approach the block and check if it is a wooden or foam block,
	 * if it is a wooden block go back and continue scanning for blocks
	 * if it is a blue block, pick it up and place or stack it
	 */
	public void getBlock(double endingAngle, double angleOfBlock, int cornerX, int cornerY) {
		boolean hasBlock = false;

		// go forward towards block
		ActionController.setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, true);
		
		// keep checking if there is a block ahead
		while (true) {
			// when there is a block ahead break out of the loop
			if (frontUsPoller.isBlock()) {
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
				
				double distance1 = frontUsPoller.getClippedData(255);
				
				// delay for a bit
		        Delay.msDelay(Constants.DELAY_MS);
				
				double distance2 = frontUsPoller.getClippedData(255);
				
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
	
	/**
	 * Search for blocks
	 * @param corners a Point array of each of the corners of the zone to search
	 * 
	 */
	public void search(Point[] corners) {
		// a certain amount up from the corner so that it is not exactly in the corner
		int xBuffer = (int) (Constants.DISTANCE_FROM_CORNER * Math.sin(45));
		int yBuffer = (int) (Constants.DISTANCE_FROM_CORNER * Math.cos(45));
		
	     HashMap<String, HashMap<String, Integer>> cornersAndAngles = new HashMap<String, HashMap<String, Integer>>();

	     HashMap<String, Integer> lowerLeft = new HashMap<String, Integer>();
	     lowerLeft.put("x", (int)corners[0].x - xBuffer);
	     lowerLeft.put("y", (int)corners[0].y - yBuffer);
	     lowerLeft.put("angle", 90 + Constants.STARTING_SCANNING_ANGLE);

	     HashMap<String, Integer> lowerRight = new HashMap<String, Integer>();
	     lowerRight.put("x", (int)corners[1].x + xBuffer);
	     lowerRight.put("y", (int)corners[1].y - yBuffer );
	     lowerRight.put("angle", 180 + Constants.STARTING_SCANNING_ANGLE);

	     HashMap<String, Integer> upperLeft = new HashMap<String, Integer>();
	     upperLeft.put("x", (int)corners[2].x - xBuffer);
	     upperLeft.put("y", (int)corners[2].y + yBuffer);
	     upperLeft.put("angle", Constants.STARTING_SCANNING_ANGLE);

	     HashMap<String, Integer> upperRight = new HashMap<String, Integer>();
	     upperRight.put("x", (int)corners[3].x + xBuffer);
	     upperRight.put("y", (int)corners[3].y + yBuffer);
	     upperRight.put("angle", 270 + Constants.STARTING_SCANNING_ANGLE);
	     
	     cornersAndAngles.put("lowerLeft", lowerLeft);
	     cornersAndAngles.put("lowerRight", lowerRight);
	     cornersAndAngles.put("upperLeft", upperLeft);
	     cornersAndAngles.put("upperRight", upperRight);
	     
	     // calculate the degrees to scan
	     double degreesToScan = 270 - (2 * Constants.STARTING_SCANNING_ANGLE);

	     // for each corner of the zone search for blocks
	     for (HashMap<String, Integer> corner : cornersAndAngles.values()) {
	    	 // travel to the corner
	    	 navigator.travelTo(corner.get("x"), corner.get("y"));
	    	 
	    	 // turn to starting angle of that corner
	    	 navigator.turnTo(corner.get("angle"));
	    	 
	    	 // For testing only
	 		Button.waitForAnyPress();

	    	 // get the angle to stop scanning at
	    	 double endingAngle = corner.get("angle") + degreesToScan;
	    	 // wrap ending angle to not be over 360 degrees
	    	 endingAngle = navigator.wrapAngle(endingAngle);

	    	 // start scanning for blocks
	    	 scanForBlocks(endingAngle, corner.get("x"), corner.get("y"));

	     }

		// once it scanned all the corners
		// TODO implement secondary searching steps here
	}

	@Override
	public void timedOut() {
		
//		//Navigation mode
//		if(!lightPoller.isLine())
//		{
			//Block detected mode
			if(frontUsPoller.isBlock())
			{
				if(lightPoller.isBlue())
				{
					//Claw pickup routine
					claw.pickUpBlock();
				}
				else
				{
					//Obstacle avoidance
				}
			}
			//Navigation mode
//			else
//			{
////				navigator.travelTo(LGZx, LGZy); //Goes to greenzone
//			}
//		}
//		else
//		{
//			//Odometry correction mode
//		}

			else {
				

		}
		
			

		// TODO EVERYTHING!!!
	//	setWifiInfo();
		
		// just for testing, actually get the values from the wifi
	//	double greenAreaX = 60;
	//	double greenAreaY = 60;

		//startOdometer();
		
		
	//	ClawController claw = new ClawController(clawLift, clawClose);
	//	ObstacleAvoider avoider = new ObstacleAvoider();

		//start USPoller timelistener here?
		/*		
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
		
		// avoid getting into while loop right away
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
		claw.placeBlock(false);*/

		
	}
}
