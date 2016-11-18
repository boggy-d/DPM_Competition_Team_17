package component;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;

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

	public static final Port lightPort = LocalEV3.get().getPort("S1");
	public static final SensorModes lightSensor = new EV3ColorSensor(lightPort);

	public static final Port colorPort = LocalEV3.get().getPort("S3");
	public static final SensorModes colorSensor = new EV3ColorSensor(colorPort);
	
	
	//motor speeds
	public final static int FAST_FORWARD_SPEED = 200;
	public final static int FORWARD_SPEED = 100;
	public final static int FAST_ROTATION_SPEED = 100;
	public final static int ROTATION_SPEED = 60;
	public final static int ACCELERATION = 3000;
	
	//navigator
	public final static double TRACK = 14.2;
	public final static double RADIUS = 2.04;
	public final static int DIST_ERR = 0000;
	
	//us localization
	public final static int ROTATE_SPEED = 100, CLIP = 45, WALL_DIST = 42, US_MARGIN = 2;
	
	//for lightLocalization 
	public final static double COLOR_DIST = 14.5;									//distance between the center of rotation of robot and light sensor
	public final static double BUFFER_DIST = 6/Math.sqrt(2);						//buffer distance (where we want to be before rotating)
	
	// odometery correction
	public final static double SENSOR_DISTANCE = 0000;
	public final static double TILE_LENGTH = 30.48; //odometer must be verified that values in x&y are scaled to cm with double precision 

	// light Poller
	public final static double BLACKINTENSITY = 0.2;
	public final static double BLUECOLOURID = 2;
	public final static double LINE_DETECT_DIFF = 0.3;
	
	// us poller
	public final static double BLOCK_INFRONT = 3;

	// odometer
	public final static int DEFAULT_TIMEOUT_PERIOD = 20;

	// claw controller
	public final static int CLAW_LIFT_FULL = (int) (360 * 2.25);
	public final static int CLAW_LIFT_ONE_BLOCK = -700;
	public final static int CLAW_LIFT_TWO_BLOCK = 600;
	public final static int CLAW_LIFT_THREE_BLOCK = 500;
	public final static int CLAW_CLOSE_ANGLE = 75;
	public final static int CLAW_OPEN_ANGLE = 60;

	
	// searching
	public final static int SEARCH_DISTANCE_THRESHOLD = 200;

	public final static String SERVER_IP = "192.168.2.3";
	public final static int TEAM_NUMBER = 17;
}
 