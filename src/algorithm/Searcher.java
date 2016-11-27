/**
 * The Searcher class looks for blocks in the arena, and places them properly in the zone
 * 
 * @author Eva Suska
 * @version 1.0.0
 * @since 0.1.0
 */

package algorithm;

import java.util.HashMap;

import component.ActionController;
import component.Constants;
import component.Odometer;
import component.USPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.geometry.Point;
import lejos.utility.Delay;

public class Searcher{
	Point[] zone;	
	Point[] restrictedZone;	
	Point blockLocation;
	int maxTowerHeight;
	int towerHeight;

	public Searcher(Point[] zone, Point[] restrictedZone, int maxTowerHeight) {
		this.zone = zone;
		this.restrictedZone = restrictedZone;
		this.maxTowerHeight = maxTowerHeight;

		// initialize starting block placing position
		blockLocation = new Point((float)(zone[0].x + Constants.DELTA_X), (float)(zone[0].y + Constants.TILE_LENGTH / 4));
		towerHeight = 0;
	}

	/**
	 * Search for blocks
	 * @param corners a Point array of each of the corners of the zone to search
	 * 
	 */
	public void search() {
		// a certain amount up from the corner so that it is not exactly in the corner
		int xBuffer = (int) (Constants.DISTANCE_FROM_CORNER * Math.sin(45));
		int yBuffer = (int) (Constants.DISTANCE_FROM_CORNER * Math.cos(45));

		// create a hashmap of the position the robot should be at for each corner and the angle it should turn to fo each corner
		HashMap<String, HashMap<String, Integer>> cornersAndAngles = new HashMap<String, HashMap<String, Integer>>();

		HashMap<String, Integer> lowerLeft = new HashMap<String, Integer>();
		lowerLeft.put("x", (int)zone[0].x - xBuffer);
		lowerLeft.put("y", (int)zone[0].y - yBuffer);
		lowerLeft.put("angle", 90 + Constants.STARTING_SCANNING_ANGLE);

		HashMap<String, Integer> lowerRight = new HashMap<String, Integer>();
		lowerRight.put("x", (int)zone[1].x + xBuffer);
		lowerRight.put("y", (int)zone[1].y - yBuffer );
		lowerRight.put("angle", 180 + Constants.STARTING_SCANNING_ANGLE);

		HashMap<String, Integer> upperRight = new HashMap<String, Integer>();
		upperRight.put("x", (int)zone[3].x + xBuffer);
		upperRight.put("y", (int)zone[3].y + yBuffer);
		upperRight.put("angle", 270 + Constants.STARTING_SCANNING_ANGLE);

		HashMap<String, Integer> upperLeft = new HashMap<String, Integer>();
		upperLeft.put("x", (int)zone[2].x - xBuffer);
		upperLeft.put("y", (int)zone[2].y + yBuffer);
		upperLeft.put("angle", Constants.STARTING_SCANNING_ANGLE);

		cornersAndAngles.put("lowerLeft", lowerLeft);
		cornersAndAngles.put("lowerRight", lowerRight);
		cornersAndAngles.put("upperRight", upperRight);
		cornersAndAngles.put("upperLeft", upperLeft);

		// calculate the degrees to scan
		double degreesToScan = 270 - (2 * Constants.STARTING_SCANNING_ANGLE);

		// for each corner of the zone search for blocks
		for (HashMap<String, Integer> corner : cornersAndAngles.values()) {

			// TODO use zone avoiding algorithm (wavefront ect) when traveling to the corner
			// travel to the corner
			ActionController.navigator.travelTo(corner.get("x"), corner.get("y"));

			// turn to starting angle of that corner
			ActionController.navigator.turnTo(corner.get("angle"));

			// get the angle to stop scanning at
			double endingAngle = corner.get("angle") + degreesToScan;
			// wrap ending angle to not be over 360 degrees
			endingAngle = ActionController.navigator.wrapAngle(endingAngle);

			// For testing only
			Button.waitForAnyPress();
			
			// start scanning for blocks
			scanForBlocks(endingAngle, corner.get("x"), corner.get("y"));
		}

		// once it scanned all the corners
		// TODO implement secondary searching steps here
	}

	/**
	 * Scan for blocks
	 * @param endingAngle angle the robot should stop scanning at
	 * @param cornerX the x coordinate of the corner you are searching
	 * @param cornerY the y coordinate of the corner you are searching
	 */
	public void scanForBlocks(double endingAngle, int cornerX, int cornerY) {		
		// scan until it reaches ending angle
		while (ActionController.odometer.getAng() < endingAngle) {
			// start rotating counter clockwise
			ActionController.setSpeeds(-Constants.ROTATION_SPEED, Constants.ROTATION_SPEED, true);

			double distance = ActionController.frontUsPoller.getClippedData(255);
			Point blockPosition = ActionController.calculatePosition(ActionController.odometer.getPosition(), distance);

			// if the distance is less than the distance the sensor can see and the block is not out of bounds (a wall)
			if (distance < Constants.SEARCH_DISTANCE_THRESHOLD & ActionController.inBounds(blockPosition)) {
				
//				// test this delay to see what is a good time
//				// delay to make it face the block, not just the edge
//				Delay.msDelay(500);

				// For testing only
				Sound.beep();
				
				// once it sees a block stop
				ActionController.stopMotors();

				// is is a block, check what block it is 
				getBlock(endingAngle, ActionController.odometer.getAng(), cornerX, cornerY);
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
	 * @param endingAngle the angle the robot should stop scanning at
	 * @param angleOfBlock the angle where the robot detected an object
	 * @param cornerX the x coordinate of the corner you are searching
	 * @param cornerY the y coordinate of the corner you are searching
	 */
	public void getBlock(double endingAngle, double angleOfBlock, int cornerX, int cornerY) {
		boolean hasBlock = false;

		// go forward towards block
		ActionController.setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, true);

		// keep checking if there is a block ahead
		while (true) {
			// when there is a block ahead break out of the loop
			if (ActionController.frontUsPoller.isBlock()) {
				ActionController.stopMotors();
				break;
			} else if (ActionController.frontUsPoller.getClippedData(255) < Constants.SEARCH_DISTANCE_THRESHOLD ) {
				// go forward towards block
				ActionController.setSpeeds(Constants.FORWARD_SPEED, Constants.FORWARD_SPEED, true);
			} else {
				// keep rotating counter clockwise towards the block
				ActionController.setSpeeds(-Constants.ROTATION_SPEED, Constants.ROTATION_SPEED, true);
				// delay to make it face the block, not just the edge
				Delay.msDelay(200);
			}
		}

		// if it is a blue block pick it up
		if (ActionController.lightPoller.isBlue()) {
			//Claw pickup routine
			ActionController.claw.pickUpBlock();
			hasBlock = true;
		}

		// if it has a block place it
		if (hasBlock) {		
			// if the tower height is equal to the max tower height, get the next position it should be in
			if (towerHeight >= maxTowerHeight) {
				chooseNextBlockPosition();
			}

			// go to the position of where the block should be placed
			// TODO use restricted zone navigating pattern instead of travelTo
			ActionController.navigator.travelTo(blockLocation.x, blockLocation.y);

			// always face the same direction when placing blocks
			ActionController.navigator.turnTo(180);

			if (towerHeight + 1 < maxTowerHeight) {
				// stack block
				ActionController.claw.placeBlock(true);
			} else {
				// don't stack block
				ActionController.claw.placeBlock(false);
			}

			towerHeight++;

			// TODO add block position we just placed to zones we should avoid


			// TODO use zone avoiding algorithm (wavefront ect) when traveling to the corner
			// go back to the corner
			ActionController.navigator.travelTo(cornerX, cornerY);
		} else {
			// TODO use zone avoiding algorithm (wavefront ect) when traveling to the corner
			// go back to the corner
			ActionController.navigator.travelTo(cornerX, cornerY);

			// if it doesn't have block continue scanning where it left off
			ActionController.navigator.turnTo(angleOfBlock);

			// don't start scanning until you've passed the obstacle
			while (true) {
				// start rotating counter clockwise
				ActionController.setSpeeds(-Constants.ROTATION_SPEED, Constants.ROTATION_SPEED, true);

				double distance1 = ActionController.frontUsPoller.getClippedData(255);

				// delay for a bit
				Delay.msDelay(Constants.DELAY_MS);

				double distance2 = ActionController.frontUsPoller.getClippedData(255);

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
	 * Gets the next position for the block to be placed
	 */
	public void chooseNextBlockPosition() {		
		// add y offset
		double newY = blockLocation.y + Constants.DELTA_Y;

		// if it is within the zone
		/* could also add check to see if it is within the x coordinates also
		 * but i don't think we are going to get to that many blocks 
		 * / then what do we do if we run out of space */
		if (newY < zone[3].y) {
			// update to the next location above
			blockLocation.setLocation(newY, blockLocation.y);
		} else {
			// if it outside the zone create a new column, add offset to X, set y to initial y block position
			blockLocation.setLocation(blockLocation.x + Constants.DELTA_X, zone[0].y + Constants.TILE_LENGTH / 4);
		}

		// set towerHeight to 0 since we are starting new positions
		towerHeight = 0;
	}
}
