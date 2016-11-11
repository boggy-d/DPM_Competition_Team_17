/**
 * The Odometer class uses the motors tachometer to
 * get the position and heading of the robot with
 * respect to the field's grid.
 * 
 * The odometer defines cooridinate system as such...
 * 
 * 					90Deg:pos y-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 					270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 * @version 1.0.0
 * @since 0.1.0
 */

package component;

import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;
	
	/**
	 * Class constructor specifying motors, thread refresh rate, and the state of the thread
	 * @param leftMotor the robot's left motor
	 * @param rightMotor the robot's right motor
	 * @param INTERVAL the refresh rate of the timer thread
	 * @param autostart the state of the thread (on or off)
	 * @see Timer
	 * @see TimerListener
	 */
	public Odometer (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL, boolean autostart) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		// default values, modify for your robot
		this.rightRadius = 2.178; 
		this.leftRadius = 2.178; 
		this.width = 16.18;
		
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}
	
	/**
	 * Stops the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}
	
	/**
	 * Starts the Timer
	 * @see Timer
	 * @see TimerListener
	 */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}
	
	/*
	 * Calculates displacement and heading as title suggests
	 */
	/**
	 * Calculates displacement and heading using the motors' tachometer
	 * values
	 * @param data
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}
	
	@Override 
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	/**
	 * Returns the X coordinate of the robot according to the odometer
	 * @return the value of <code>x</code>
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * Returns the Y coordinate of the robot according to the odometer
	 * @return the value of <code>y</code>
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * Returns the heading of the robot according to the odometer
	 * @return the value of <code>theta</code>
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * Updates the position and/or heading of the odometer
	 * @param position contains the new values of the position and heading
	 * @param update contains the data specifying which fields should be updated
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * Updates t
	 * @param position
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * 
	 * @return
	 */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}
	
	/**
	 * Gets the robots motors
	 * @return the robot's motors
	 */
	public EV3LargeRegulatedMotor [] getMotors() {
		return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}
	
	/**
	 * Gets the robot's left motor
	 * @return the robot's left motor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	
	/**
	 * Gets the robot's right motor
	 * @return the robot's right motor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * Wraps the heading from 0 to 360
	 * @param angle the angle to be wrapped
	 * @return the angle wrapped between 0 and 360
	 */
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	/**
	 * Calculates the minimum angle to turn from angle <code>a</code>
	 * to angle <code>b</code>
	 * @param a the initial angle
	 * @param b the final angle
	 * @return the minimum angle from <code>a</code> to <code>b</code>
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
}
