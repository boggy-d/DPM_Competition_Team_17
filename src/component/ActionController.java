/**
 * The ActionController class acts as a finite state
 * machine for the system as it controls the flow
 * of the routine.
 * 
 * @author Bogdan Dumitru, Eric Zimmermann, Eva Suska
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
import algorithm.OdometerCorrection;
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

public class ActionController {

	//Instantiate more objects
	public static Odometer odometer;
	public static Navigator navigator;
	public static USPoller usPoller;
	public static LightPoller lightPoller;
	public static ClawController claw;
	public static Searcher searcher;
	public static ObstacleAvoider avoider;
	public static TimeRecorder timeRecorder;
	public static OdometerCorrection odometerCorrection;

	Point[] zone;	
	static Point[] restrictedZone;	
	Point blockLocation;
	int maxTowerHeight;
	int towerHeight;
	int i;
	double destX, destY;
	
	private Timer acTimer;
	static boolean isSearching = false;
	
	//Create competition variables
	static int SC, ROLE, LRZx, LRZy, URZx, URZy, LGZx, LGZy, UGZx, UGZy;
	
	/**
	 * Class constructor that does localization and sets up the thread loop
	 */
	public ActionController()
	{
		//Create odometer and sensors before gtting wifi info
		odometer = new Odometer(30, true, 0, 0, 0);
		usPoller = new USPoller(Constants.frontUsSensor, Constants.sideUsSensor);
		lightPoller = new LightPoller(Constants.lightSensor, Constants.colorSensor);
		
		setWifiInfo();
		
		//Starts round time
		timeRecorder = new TimeRecorder();
		
		//Instantiates rest of objects
		claw = new ClawController();
		avoider = new ObstacleAvoider();
		navigator = new Navigator();
		
		//Localize to corner
		USLocalizer usLocalizer = new USLocalizer();
		LightLocalizer lightLocalizer = new LightLocalizer();
		usLocalizer.usLocalize();
		lightLocalizer.lightlocalize();
		ActionController.navigator.travelTo(0,0);
		ActionController.navigator.turnTo(0);

		//Updates odometer position based on starting corner
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

		jobAssigned();
		//Starts odometry correction
		odometerCorrection = new OdometerCorrection(25, true);
		
		//Initializes the searching algorithm
		searcher = new Searcher(zone, restrictedZone, maxTowerHeight);
		
		//Set the position of the first corner to travel to
		i = 0;
		destX = searcher.cornersAndAngles.get(i).get("x");
		destY = searcher.cornersAndAngles.get(i).get("y");
		navigator.partitionedPathTravelTo(this.destX, this.destY, Constants.MOVEMENT_PARTITIONS);
	}

	/**
	 * Sets the speeds of the motors and if they
	 * should be activated or not
	 * 
	 * @param lSpd	the speed of the left motor
	 * @param rSpd	the speed of the right motor
	 * @param move true if the robot starts moving, false if not
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
	 * Gets the competition information provided by WiFi
	 * and stores it into fields
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
	 * Reads the <code>BTN</code> and <code>CTN</code> and returns the
	 * robots job for the heat
	 */
	public void jobAssigned() {
		// Decide the role of the robot based on wifi info
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
	public static void goToStart() {		
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
	}

	/**
	 * Main routine of robot
	 */
	public void doRoutine() {
		// Time is still remaining, do routine
		if (timeRecorder.isTimeRemaining()) {
			// Travel to not done, keep going to POI
			if (navigator.movementCounter != Constants.MOVEMENT_PARTITIONS  || isSearching )
																			// (this
																			// flag
																			// overrides
																			// the
																			// need
																			// to
																			// not
																			// be
																			// at
																			// destination
																			// to
																			// check
																			// for
																			// blocks
																			// (basically
																			// makes
																			// it
																			// work
																			// when
																			// searching
																			// too)
			{
				// Object in front detected
				if (usPoller.isFrontBlock()) {
					// Robot doesn't currently have a block grabbed
					if (!claw.isBlockGrabbed()) {
						// Search mode needs to constantly moves motor, navigate
						// mode only sets speeds cause movements done by
						// partitions
						if (isSearching) {
							setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, true); // Get closer slower to detect color better
							
						} else {
							setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, false); // Set the motors to a lower speed
						}

						// Block is blue, pick it up
						if (lightPoller.isBlue()) {
							System.out.println("BLUE OK!");
							claw.pickUpBlock(); // WHAT IF WE GO BACKWARDS DOES
												// THE PPTT STILL WORK

							searcher.chooseNextBlockPosition();
							destX = searcher.blockLocation.x;
							destY = searcher.blockLocation.y;
							navigator.partitionedPathTravelTo(this.destX, this.destY, Constants.MOVEMENT_PARTITIONS);
							
							isSearching = false;
						}

						// Not a blue block, obstacle avoidance
						else {
							//avoider.avoidObstacle();
						}
					}

					// The claw already has a block, obstacle avoidance
					else {
						//avoider.avoidObstacle();
					}
				}

				// Normal navigation, keep navigating to POI
				else if (!isSearching) // makes it so movement uses pPTT when
										// not searching and setSpeeds when
										// searching
				{
					Constants.leftMotor.rotate((int) navigator.dR, true);
					Constants.rightMotor.rotate((int) navigator.dR, false);

					 setSpeeds(Constants.FORWARD_SPEED,Constants.FORWARD_SPEED, true);

					++navigator.movementCounter;
				}

				// Search mode navigation, keep going towards object
				else {

					while (ActionController.usPoller.getFrontDistance(Constants.SEARCHING_CLIP) < Constants.SEARCH_DISTANCE_THRESHOLD
							&& ActionController.inBounds(ActionController.calculatePosition(ActionController.odometer.getPosition(), ActionController.usPoller.getFrontDistance(Constants.SEARCHING_CLIP))))
					{
						setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, true);
						if(ActionController.usPoller.getFrontDistance(255) < 8)
						{ stopMotors();
						break;
						}
					}
					
					if(ActionController.usPoller.getFrontDistance(255) < 8)
					{
						// keep rotating counter clockwise towards the block
						goForward(3, Constants.FORWARD_SPEED);
						Delay.msDelay(200);
						if (lightPoller.isBlue())
						{
							claw.pickUpBlock();
						}
						else
						{
							ActionController.setSpeeds(-Constants.SLOW_ROTATION_SPEED, Constants.SLOW_ROTATION_SPEED, true);
							Delay.msDelay(200);
							ActionController.stopMotors();
						}
						
						
						
					}
					
				}
			}

			// Travel done, either search or place block
			else {
				// ALGORITHM, MUST DISCUSS
				if (claw.isBlockGrabbed()) {
					// Detected an existing block where the block is supposed to
					// be placed, stack
					if (usPoller.isFrontBlock() && lightPoller.isBlue()) {
						claw.placeBlock(true);
					}
					// No block where the block is supposed to be placed, place
					// block on ground
					else {
						claw.placeBlock(false);
					}
					// Go back to current corner
					destX = searcher.cornersAndAngles.get(i).get("x");
					destY = searcher.cornersAndAngles.get(i).get("y");
					navigator.partitionedPathTravelTo(this.destX, this.destY, Constants.MOVEMENT_PARTITIONS);
				}

				// We're at one of the corners, do scanForBlocks
				else {
					
					// turn to starting angle of that corner
					ActionController.navigator.turnTo(searcher.cornersAndAngles.get(i).get("angle"));

					// get the angle to stop scanning at
					double endingAngle = searcher.cornersAndAngles.get(i).get("angle") + searcher.degreesToScan;
					// wrap ending angle to not be over 360 degrees
					endingAngle = ActionController.navigator.wrapAngle(endingAngle);
					
					// start scanning for blocks
					if(searcher.scanForBlocks(endingAngle, searcher.cornersAndAngles.get(i).get("x"), searcher.cornersAndAngles.get(i).get("y")))
					{
						isSearching = true;
					}
					else
					{
						i++; //(counter for corners[i])
						destX = searcher.cornersAndAngles.get(i).get("x");
						destY = searcher.cornersAndAngles.get(i).get("y");
						navigator.partitionedPathTravelTo(this.destX, this.destY, Constants.MOVEMENT_PARTITIONS);
					}
				}
			}
		}

		// Round almost over, stop everything and go to start
		else {
			// Obstacle avoidance on any block
			if (usPoller.isFrontBlock()) {
				//avoider.avoidObstacle();
			}
			// Move to starting position
			else {
				goToStart();
			}

		}
	}
}
