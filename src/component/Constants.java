package component;

public class Constants {
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
 