/**
 * System's constants
 */
package component;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;

public class Constants {
	
	// Instantiate motors
	public final static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public final static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public final static EV3LargeRegulatedMotor clawLift = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public final static EV3LargeRegulatedMotor clawClose = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	// Instantiate sensors
	// private static final EV3UltrasonicSensor frontUsSensor = new
	// EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));

	public static final Port usPort = LocalEV3.get().getPort("S2");
	public static final SensorModes frontUsSensor = new EV3UltrasonicSensor(usPort);

	public static final Port us2Port = LocalEV3.get().getPort("S4");
	public static final SensorModes sideUsSensor = new EV3UltrasonicSensor(us2Port);
	
	public static final Port lightPort = LocalEV3.get().getPort("S1");
	public static final SensorModes lightSensor = new EV3ColorSensor(lightPort);

	public static final Port colorPort = LocalEV3.get().getPort("S3");
	public static final SensorModes colorSensor = new EV3ColorSensor(colorPort);
	
	public final static int MOVEMENT_PARTITIONS = 0;
	
	//motor speeds
	public final static int FAST_FORWARD_SPEED = 250;
	public final static int FORWARD_SPEED = 150;
	public final static int FAST_ROTATION_SPEED = 200;
	public final static int ROTATION_SPEED = 130;
	public final static int SLOW_ROTATION_SPEED = 60;
	public final static int ACCELERATION = 3000;
	public final static int AVOID_SPEED = 100;
	
	//navigator
	public final static double TRACK = 13.8;
	public final static double RADIUS = 2.04;
	public final static double DIST_ERR = 1.5;
	
	//us localization
	public final static int ROTATE_SPEED = 300;
	public final static int LOCALIZATION_CLIP = 45;
	public final static int WALL_DISTANCE_THRESHOLD = 42;
	public final static int US_MARGIN = 2;
	
	//for lightLocalization 
	public final static double COLOR_DIST = 14.5;				//distance between the center of rotation of robot and light sensor
	public final static double BUFFER_DIST = 9.5;				//buffer distance (where we want to be before rotating)
	
	// odometery correction
	public final static double SENSOR_DISTANCE = 14.5;
	public final static double TILE_LENGTH = 30.48; 
	public final static double CORNER_MARGIN = 0.15;
	
	// light Poller
	public final static double BLACKINTENSITY = 0.2;
	public final static double BLUECOLOURID = 2;
	public final static double LINE_DETECT_DIFF = 0.3;
	
	// us poller
	public final static double BLOCK_INFRONT = 8;

	// odometer
	public final static int DEFAULT_TIMEOUT_PERIOD = 20;

	// claw controller
	public final static int CLAW_LIFT_FULL = (int) (360 * 2.25);
	public final static int CLAW_STACK = (int) (360);
	public final static int CLAW_CLOSE_ANGLE = 90;
	public final static int CLAW_OPEN_ANGLE = 80;

	// searching
	public final static int SEARCH_DISTANCE_THRESHOLD = 60;
	public final static int STARTING_SCANNING_ANGLE = 30;
	public final static int DELAY_MS = 500;
	public final static int DISTANCE_DIFFERENCE = 5;
	public final static int DISTANCE_FROM_CORNER = 10;
	public final static int SCAN_MARGIN = 5;
	public final static int OUT_OF_BOUNDS_MARGIN = 5;
	public final static int SEARCHING_CLIP = 255;

	//obstacle avoidance
	public final static int AVOID_SPEED_STRAIGHT = 100;
	public final static int AVOID_SPEED_LOW = 70;
	public final static int AVOID_SPEED_HIGH = 130;
	public final static int BANDCENTER = 30;
	public final static int BANDWIDTH = 3;
	public static final double CORNER_ANGLE_ERROR = Math.toRadians(2); //Threshold value for when robot returns to angle before bang bang, hence it cleared to wall
	public static final double FINISH_ANGLE_ERROR = Math.toRadians(89.5); //Threshold value between pre-bangbang angle and finish angle (should be 90)

	// block placing
	public final static double DELTA_X = TILE_LENGTH/2;
	public final static double DELTA_Y = TILE_LENGTH/2;
	
	// go back to start
	public final static double DISTANCE_IN_CORNER = 15;


	public final static String SERVER_IP = "192.168.2.3";
	public final static int TEAM_NUMBER = 17;
	
	public static final int LCD_REFRESH = 100;
	
	public static final long FIVE_MINUTES = 300000;
	public static final long TIME_TO_GO_TO_START = 60000;
	
	
	
}
 
