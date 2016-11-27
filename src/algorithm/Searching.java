package algorithm;

import java.util.HashMap;

import component.ActionController;
import component.Constants;
import lejos.hardware.Button;
import lejos.robotics.geometry.Point;

public class Searching {
	
	HashMap<String, HashMap<String, Integer>> cornersAndAngles;
	
	/**
	 * Class constructor that stores all the corners of the zone
	 * in a HashMap
	 * @param corners the corners of a zone
	 */
	public Searching(Point[] corners)
	{
		this.cornersAndAngles = new HashMap<String, HashMap<String, Integer>>();
		
	    HashMap<String, Integer> lowerLeft = new HashMap<String, Integer>();
	    lowerLeft.put("x", (int)corners[0].x);
	    lowerLeft.put("y", (int)corners[0].y);
	    lowerLeft.put("angle", 90 + Constants.STARTING_SCANNING_ANGLE);

	    HashMap<String, Integer> lowerRight = new HashMap<String, Integer>();
	    lowerRight.put("x", (int)corners[1].x);
	    lowerRight.put("y", (int)corners[1].y);
	    lowerRight.put("angle", 180 + Constants.STARTING_SCANNING_ANGLE);

	    HashMap<String, Integer> upperLeft = new HashMap<String, Integer>();
	    upperLeft.put("x", (int)corners[2].x);
	    upperLeft.put("y", (int)corners[2].y);
	    upperLeft.put("angle", Constants.STARTING_SCANNING_ANGLE);

	    HashMap<String, Integer> upperRight = new HashMap<String, Integer>();
	    upperRight.put("x", (int)corners[3].x);
	    upperRight.put("y", (int)corners[3].y);
	    upperRight.put("angle", 270 + Constants.STARTING_SCANNING_ANGLE);
	     
	    this.cornersAndAngles.put("lowerLeft", lowerLeft);
	    this.cornersAndAngles.put("lowerRight", lowerRight);
	    this.cornersAndAngles.put("upperLeft", upperLeft);
	    this.cornersAndAngles.put("upperRight", upperRight);
	}
	
	
	/**
	 * TBD
	 */
	public void search()
	{
	     // calculate the degrees to scan
	     double degreesToScan = 270 - (2 * Constants.STARTING_SCANNING_ANGLE);

	     // for each corner of the zone search for blocks
	     for (HashMap<String, Integer> corner : cornersAndAngles.values()) {
	    	 // travel to the corner
	    	 ActionController.navigator.travelTo(corner.get("x"), corner.get("y"));

	    	 // turn to starting angle of that corner
	    	 ActionController.navigator.turnTo(corner.get("angle"));
	    	 
	 		Button.waitForAnyPress();

	    	 // get the angle to stop scanning at
	    	 double endingAngle = corner.get("angle") + degreesToScan;
	    	 // wrap ending angle to not be over 360 degrees
	    	 endingAngle = ActionController.navigator.wrapAngle(endingAngle);
	     }

		// start scanning for blocks
		scanForBlocks(endingAngle, corner.get("x"), corner.get("y"));
	}

	/**
	 * Searches the 4 corners of the zone
	 * 
	 * @param corners the corners of the zones
	 */
	public void scanForBlocks(Point[] corners) {
	     
	    	 // start scanning for blocks
	    	scanForBlocks(endingAngle, corner.get("x"), corner.get("y"));


		// once it scanned all the corners
		// TODO implement secondary searching steps here
	}
	
	/**
	 * Uses searching algorithm to scan for blocks
	 * 
	 * @param endingAngle
	 * @param cornerX
	 * @param cornerY
	 */
	public void scanForBlocks(double endingAngle, int cornerX, int cornerY) {		
		// scan until it reaches ending angle
		
		Point blockPosition = calculatePosition(odometer.getPosition(), usPoller.getFrontDistance());
		ActionController.setSpeeds(-Constants.ROTATION_SPEED, Constants.ROTATION_SPEED, true);		
		
		// if the distance is less than the distance the sensor can see and the block is not out of bounds (a wall)
		if (ActionController.usPoller.isFrontBlock() && inBounds(blockPosition)) {
			// once it sees a block stop
			ActionController.stopMotors();
			
			// is is a block, check what block it is 
			getBlock(endingAngle, ActionController.odometer.getAng(), cornerX, cornerY);
		}
	}
}
