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

public class ObstacleAvoider{
	private int distance = 0;
	private Timer obstacleTimer;
	
	private double initialTheta = 0; //The angle the robot detects the wall at
	private double currentTheta;
	private boolean isCleared = false;
	private boolean isDone = false;
	//Default Constructor
	
	
	public ObstacleAvoider()
	{
		
		initialTheta = ActionController.odometer.getAng();
		//turn robot 90deg to right then move forward 
		ActionController.navigator.turnTo(ActionController.navigator.wrapAngle(ActionController.odometer.getAng() - 90));
		ActionController.setSpeeds(Constants.AVOID_SPEED_STRAIGHT, Constants.AVOID_SPEED_STRAIGHT,true);
		
	}
	
	/**
	 * Avoids blocks using Bang bang algorithm
	 * 
	 * @see USPoller
	 * @see TimeRecorder
	 * @see Odometer
	 */
	public void avoidObstacle()
	{
		while(!isDone)
		{
			currentTheta = ActionController.odometer.getAng() % 360;

			// If the corner has been cleared and the angle is about 90 deg from the
			// pre-bangbang angle stop wall-following
			if (isCleared && Math.abs(initialTheta - currentTheta) > Constants.FINISH_ANGLE_ERROR) {
				ActionController.stopMotors();
				isCleared = false;
				isDone = true;
			}

			// If the pre-bangbang angle is similar to the angle from the odometer
			// during wall-following
			// the corner has been cleared
			else if ((Math.abs(initialTheta - currentTheta)) <= Constants.CORNER_ANGLE_ERROR) {
				isCleared = true;
			}

			// Else do bangbang
			else {
				bangBang();
			}
		}
		isDone = false;
	}

	/**
	 * Avoids walls by turning left motor faster if the robot gets close to a wall or
	 * turning the right motor faster if it goes too far away
	 */
	public void bangBang() {
		// Processes a movement based on the US passed
		// distance is within the bandCenter, it moves straight
		if (ActionController.usPoller.getSideDistance() <= Constants.BANDCENTER + Constants.BANDWIDTH
				&& ActionController.usPoller.getSideDistance() >= Constants.BANDCENTER - Constants.BANDWIDTH) {

			ActionController.setSpeeds(Constants.AVOID_SPEED_STRAIGHT, Constants.AVOID_SPEED_STRAIGHT, true);
		}

		// too close, it needs to move away from the wall
		// left wheel fast, right wheel slow. Causes robot to move toward bandcenter (move right)
		else if (ActionController.usPoller.getSideDistance() < Constants.BANDCENTER - Constants.BANDWIDTH) {
			ActionController.setSpeeds(Constants.AVOID_SPEED_HIGH, Constants.AVOID_SPEED_LOW, true);
		}

		// too far, it needs to move closer to the wall
		// left wheel slow, right wheel fast. Causes robot to move toward bandcenter (move left)
		else if (ActionController.usPoller.getSideDistance() > Constants.BANDCENTER + Constants.BANDWIDTH) {
			ActionController.setSpeeds(Constants.AVOID_SPEED_LOW, Constants.AVOID_SPEED_HIGH, true);
		}
		
	}
}
