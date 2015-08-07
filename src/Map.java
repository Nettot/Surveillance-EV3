import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;


public class Map {
	
	private static float angle;
	private static Double cVal;
	private static Double sVal;
	private static int x;
	private static int y;
	private static EV3IRSensor ir;
	private static float range;
	static int XCENTRE = 89;
	static int YCENTRE = 64;
	//Initialize motors
	static RegulatedMotor l = new EV3LargeRegulatedMotor(MotorPort.A);
	static RegulatedMotor r = new EV3LargeRegulatedMotor(MotorPort.D);
	
	/*
	 * 
	 */
	public static void main (String[] args) {
		
		//Creates pilot
		DifferentialPilot pilot = new DifferentialPilot(1.4, 8, l, r);
		//Used to fetch the sensor and initialize distance mode
		ir = new EV3IRSensor(SensorPort.S1);
		SensorMode distance = ir.getDistanceMode();
		float[] sample = new float[distance.sampleSize()];
		
		pilot.setRotateSpeed(30);
		pilot.rotateLeft();
		
		/*
		 * This While loop will activate when the EV3 is moving and will do the bulk of the work.
		 * Loop is designed to take all of the relevant info that is needed and apply calculations
		 * so that an x & y coordinate are found which will then be printed to the screen.
		 */
		while (pilot.isMoving()) {
			angle = pilot.getAngleIncrement();
			distance.fetchSample(sample, 0);
			//LCD.drawString("" + angle, 0, 0);
			//LCD.drawString(" " + sample[0], 0, 0);
			range = sample[0];
			
			//Calculates the x & y coordinates using simple trig
			cVal = range * Math.cos(angle);
			x = cVal.intValue();
			sVal = range * Math.sin(angle);
			y = sVal.intValue();
			
			//LCD.drawString("" + x, 0, 0);
			
			//Print points to screen with offset
			LCD.setPixel(XCENTRE + x, YCENTRE + y, 1);
			
			//Stops EV3 moving more than a 360 degree turn
			if (angle > 361) {
				pilot.stop();
				Delay.msDelay(2000);
			}
			
			//Exit program
			if (Button.ESCAPE.isDown()) {
				System.exit(0);
				
			}
		}
		
		
	}
		
}


