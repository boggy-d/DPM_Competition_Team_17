/**
 * The Obstacle avoidance class runs when an object that is not
 * blue is detected by the robot's USS. The robot runs a specific
 * routine to avoid either wooden blocks or walls.
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 */

package algorithm;

import component.ActionController;
import component.Constants;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class ObstacleAvoider implements TimerListener {
	private int distance = 0;
	private Timer obstacleTimer;
	
	private double initialTheta = 0; //The angle the robot detects the wall at
	private double currentTheta;
	private boolean isCleared = false;
	//Default Constructor
	
	public ObstacleAvoider()
	{
		
		initialTheta = ActionController.odometer.getAng();
		//turn robot 90deg to right then move forward 
		ActionController.navigator.turnTo(ActionController.navigator.wrapAngle(ActionController.odometer.getAng() - 90));
		ActionController.setSpeeds(Constants.AVOID_SPEED_STRAIGHT, Constants.AVOID_SPEED_STRAIGHT,true);
		this.obstacleTimer = new Timer(Constants.LCD_REFRESH, this);
		
		obstacleTimer.start();
		
	}


	/**
	 * Checks how long the USS has been detecting values
	 * under the threshold and returns if the object is a wall
	 * 
	 * @return <code>true</code> if the USS has been detecting values under the threshold for n (TBD) seconds, <code>false</code> otherwise
	 */
	public boolean isWall()
	{
		return false;
		
	}
	
	/**
	 * Turns the robot 90deg and breaks out of obstacle avoidance (exact algorithm still TBD)
	 * 
	 * @see component.TimeRecorder
	 * @see component.Odometer
	 */
	public void avoidWall()
	{
		
	}
	
	/**
	 * Turns the robot 90deg and makes it go forward until the second
	 * USS does not detect an obstacle anymore (exact algorithm still TBD)
	 * 
	 * @see USPoller
	 * @see TimeRecorder
	 * @see Odometer
	 */
	public void avoidObstacle()
	{
		// Processes a movement based on the US passed
		// distance is within the bandCenter, it moves straight
		if (ActionController.usPoller.getSideDistance() <= Constants.BANDCENTER + Constants.BANDWIDTH
				&& distance >= Constants.BANDCENTER - Constants.BANDWIDTH) {

			ActionController.setSpeeds(Constants.AVOID_SPEED_STRAIGHT, Constants.AVOID_SPEED_STRAIGHT, true);
		}

		// too close, it needs to move away from the wall
		// outer wheel moves backwards
		else if (ActionController.usPoller.getSideDistance() < Constants.BANDCENTER - Constants.BANDWIDTH) {
			ActionController.setSpeeds(Constants.AVOID_SPEED_HIGH, Constants.AVOID_SPEED_LOW, true);
		}

		// too far, it needs to move closer to the wall
		// outer wheels moves forward faster
		else if (ActionController.usPoller.getSideDistance() > Constants.BANDCENTER + Constants.BANDWIDTH) {
			ActionController.setSpeeds(Constants.AVOID_SPEED_LOW, Constants.AVOID_SPEED_HIGH, true);
		}
		
	}

	
	@Override
	public void timedOut() {
		currentTheta = ActionController.odometer.getAng() % 360;
		
		//If the corner has been cleared and the angle is about 90 deg from the pre-bangbang angle stop wall-following
		if(isCleared && Math.abs(initialTheta - currentTheta) > Constants.FINISH_ANGLE_ERROR)
		{
			ActionController.stopMotors();
			isCleared = false;	
			this.obstacleTimer.stop();
		}
		
		//If the pre-bangbang angle is similar to the angle from the odometer during wall-following
		//the corner has been cleared
		else if((Math.abs(initialTheta - currentTheta)) <= Constants.CORNER_ANGLE_ERROR)
		{
			isCleared = true;
		}
		
		//Else do bangbang
		else
		{
			avoidObstacle();
		}
	}
}
