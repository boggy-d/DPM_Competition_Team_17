package component;

public class Constants {
	//motor speeds
	public final static int FORWARD_SPEED = 100;
	public final static int ROTATION_SPEED = 60;
	
	//for lightLocalization 
	public final static double COLOR_DIST = 13.7;									//distance between the center of rotation of robot and light sensor
	public final static double BUFFER_DIST = 6/Math.sqrt(2);						//buffer distance (where we want to be before rotating)
	
	// odometery correction
	public final static double SENSOR_DISTANCE = 0000;
	public final static double TILE_LENGTH = 30.48; //odometer must be verified that values in x&y are scaled to cm with double precision 

}
 