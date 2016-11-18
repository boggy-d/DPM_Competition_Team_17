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
	public final static int FORWARD_SPEED = 100;
	public final static int ROTATION_SPEED = 60;
	
	//us localization
	public final static int ROTATE_SPEED = 100, CLIP = 45, WALL_DIST = 42, US_MARGIN = 3;
	
	//for lightLocalization 
	public final static double COLOR_DIST = 13.7;									//distance between the center of rotation of robot and light sensor
	public final static double BUFFER_DIST = 6/Math.sqrt(2);						//buffer distance (where we want to be before rotating)
	
	// odometery correction
	public final static double SENSOR_DISTANCE = 0000;
	public final static double TILE_LENGTH = 30.48; //odometer must be verified that values in x&y are scaled to cm with double precision 

	// light Poller
	public final static double BLACKINTENSITY = 0.2;
	public final static double BLUECOLOURID = 2;
	public final static int LINE_DETECT_DIFF = 0;
	
	// odometer
	public final static int DEFAULT_TIMEOUT_PERIOD = 20;

	// claw controller
	public final static int CLAW_LIFT_FULL = 775;
	public final static int CLAW_LIFT_ONE_BLOCK = -700;
	public final static int CLAW_LIFT_TWO_BLOCK = 600;
	public final static int CLAW_LIFT_THREE_BLOCK = 500;
	public final static int CLAW_CLOSE_ANGLE = 145;
	
	public final static String SERVER_IP = "192.168.2.15";
	public final static int TEAM_NUMBER = 17;
}
 