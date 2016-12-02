
/**
 * The LightLocalization class uses the LightSensor1
 * to travel to the assigned "origin" (i.e. one of the corners)
 * on the competition's grid field
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 * @author Eva Suska
 */

package algorithm;

import component.ActionController;
import component.LightPoller;
import component.Odometer;
import component.Constants;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.utility.Delay;

public class LightLocalizer {
	
	/**
	 * Localizes the robot when it gets to the assumed (0 , 0) coordinate
	 * using the LightSensor1 to detect floor lines while doing a full circle
	 * 
	 * @see LightPoller
	 * @see Navigator
	 */
	public void lightlocalize() {
	
		//turn to approximate direction of lines
		ActionController.navigator.turnTo(45);
		
		ActionController.stopMotors();
		
		//setting speed and begin to move forward speed
		ActionController.setSpeeds(Constants.FAST_FORWARD_SPEED, Constants.FAST_FORWARD_SPEED, true);
		
	
		
		// keeps driving forward until we detect a line
		while(!(ActionController.lightPoller.isLine())) {
			
			//keep traveling forward
		}
		
		// black line detected
		ActionController.stopMotors();
			
		// move backward until we are in the negative XY quadrant
		ActionController.goForward((float)(Constants.COLOR_DIST + Constants.BUFFER_DIST), -Constants.FAST_FORWARD_SPEED);
		
		// start rotating clockwise
		ActionController.setSpeeds(Constants.ROTATION_SPEED, -Constants.ROTATION_SPEED, true);

       		 // count lines crossed and store the orientation at each line
       		 int lineCount = 0; 
		
		//angles = {angleX negative, angleY positive, angleX positive, angleY negative}
		double[] angles = new double[4] ;
		
		//line detected variable used to make sure lines are not counted twice
		boolean isOnLine = false;
		
		
		//keep rotating until 4 lines have been scanned
		while (lineCount < 4) {
			
			// keeps rotating until line is detected
			if(ActionController.lightPoller.isLine()) 
			{ isOnLine = true; }
				
			//no longer on line - read first line
				if (isOnLine) {
					
					// store angle in array
					angles[lineCount] = ActionController.odometer.getAng();
					lineCount++;
					// avoid counting same line several times
	           		 	Delay.msDelay(500);
				}
			//set variable to off line state
				isOnLine = false;
		}

		// stop motors
		ActionController.stopMotors();

        // calculate correct x, y and theta differences
        // using formulas from the tutorial slides 
        double negativeYTheta = angles[3];
        double deltaXTheta = angles[2] - angles[0];
        double deltaYTheta = angles[3] - angles[1];		
        
	//caluclate position relative to assumed (0,0)
        double deltaX = -Constants.COLOR_DIST * Math.cos(Math.toRadians(deltaYTheta) / 2);
        double deltaY = -Constants.COLOR_DIST * Math.cos(Math.toRadians(deltaXTheta) / 2);
        //calculate angle for angle correction
	double deltaTheta = 180 - negativeYTheta + deltaYTheta / 2;		
	
	//rescale angle if out of range
        if(deltaTheta > 180){
        	deltaTheta += 180;
        }
        
        // beep after localization
        Sound.beep();

     
       		// correct coordinates and orientation by updating odometer
		double[] correction = {deltaX, deltaY, ActionController.odometer.getAng() + deltaTheta};
		boolean[] update = {true, true, true};
		ActionController.odometer.setPosition(correction, update);
	}
}

