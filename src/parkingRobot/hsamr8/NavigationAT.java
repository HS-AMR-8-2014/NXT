package parkingRobot.hsamr8;

import java.util.Map;

import lejos.geom.Line;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;
import lejos.geom.Point;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception.AngleDifferenceMeasurement;
import parkingRobot.hsamr8.NavigationThread;

/**
 * A executable basic example implementation of the corresponding interface
 * provided by the Institute of Automation with limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception}
 * implementation are used for periodically calculating the robots position and
 * corresponding heading angle (together called 'pose'). Neither any use of the
 * map information or other perception sensor information is used nor any
 * parking slot detection is performed, although the necessary members are
 * already prepared. Advanced navigation calculation and parking slot detection
 * should be developed and invented by the students.
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation {

	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorRight = 0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorLeft = 0;

	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft = null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot
	 * wheel which measures the wheels angle difference between actual an last
	 * request
	 */
	IPerception.EncoderSensor encoderRight = null;

	/**
	 * reference to data class for measurement of the left wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft = null;
	/**
	 * reference to data class for measurement of the right wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight = null;

	/**
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry
	 * sensor to measure the ground displacement between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;

	/**
	 * reference to data class for measurement of the mouse odometry sensor to
	 * measure the ground displacement between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;

	/**
	 * distance from optical sensor pointing in driving direction to obstacle in
	 * mm
	 */
	double frontSensorDistance = 0;
	/**
	 * distance from optical sensor pointing to the right side of robot to
	 * obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance = 0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to
	 * obstacle in mm
	 */
	double backSensorDistance = 0;
	/**
	 * distance from optical sensor pointing to the right side of robot to
	 * obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance = 0;

	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS = 0.028; // only rough guess, to be
													// measured exactly and
													// maybe refined by
													// experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS = 0.028; // only rough guess, to be
													// measured exactly and
													// maybe refined by
													// experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE = 0.13885; // only rough guess, to be in meters  //FIXME //0.13885
	private static final String NULL = null;
	
	// measured exactly and maybe
	// refined by experiments

	/**
	 * map array of line references, whose corresponding lines form a closed
	 * chain and represent the map of the robot course
	 */
	Line[] map = null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off
	 * (false)
	 */
	boolean parkingSlotDetectionIsOn = true;
	/**
	 * pose class containing bundled current X and Y location and corresponding
	 * heading angle phi
	 */
	Pose pose = new Pose();
	int id_aktuell = 0;

	ParkingSlot[] list_ParkingSlot = new ParkingSlot[25]; // create array to
															// save ParkingSlots
															// and send them to
															// HMI

	int abstand_sens_band = 25; // distance from sensor to wall
	double phi_kontroll = 0; // a (2n)*pi/2 angle that is the angle of a line
	int akt_linie = 0; // number of actual line
	int last_linie = 0; // number of last line

	double[] listFehler = new double[200];
	int poseFehler = 0;
	double kritischerwegvorn = 12; // cm
	boolean detectionecke = false;
	double last_winkel = 0;
	double richtung = 0;
	int minimalabstand = 11;
	boolean seite_ist_was = false;
	boolean hinten_ist_was = false;
	boolean vorne_ist_was = false;
	boolean detectionactive = false;
	int abstandtriang = 4; // distance between the sensors at side
	boolean rechtskurve = false;
	boolean linkskurve = false;
	double alterwinkel = 0;
	double abstand = 0.0;
	// mouse sensor
	double mouseTime = 0.0;
	double mouseX = 0.0;
	double mouseY = 0.0;
	boolean mouseNav = true;

	boolean stateMeasurement = false;
	// slot detection
	Point anfang = new Point(0, 0);
	Point ende = new Point(0, 0);
	int id_bekannte_luecke = 0; // Nummer der Bek. L¸cke##

	boolean speichern_bereit = false;
	boolean bekannt = false; // gives information, if a slot is known or not

	boolean gueteSlot = false;
	// ParkingSlot ParkingSlot = new ParklingSlot(0, NULL, NULL, NULL);
	// get the average angle during the measurement
	float[] listWinkel = new float[200]; // check how long array has to be
	int idWinkel = 0;
	double average = 0;
	double summe = 0;
	boolean gueteMeasurement = false;
	ParkingSlotStatus slotStatus;
	boolean bmeasurement=false;
	boolean detectionline=false;
	double phiMaus=0.0;
	
	 int e=1;				// einheit Strecken
	 double we= Math.PI/180; // winkeleinheit
	 
	 boolean bekannta= false;
	 boolean bekannte=false;
	 boolean suchen=true;
	 
	Point beginpos= new Point(0,0);
	Point endpos = new Point(0,0);
	boolean onLine=false;
	boolean parkingactive=false;
	 
//	 ParkingSlot Luecke1 = new ParkingSlot(0, null, null, null);

	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * perception object (to obtain the measurement information from) and starts
	 * the navigation thread.
	 * 
	 * @param perception
	 *            corresponding main module Perception class object
	 */
	public NavigationAT(IPerception perception) {
		this.perception = perception;
		this.encoderLeft = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();

		// fill paring slot list with dummy slots
		// for (int i = 0; i < 25; i++) { //create some dummy slots for HMI to
		// avoid NullPointException
		// list_ParkingSlot[i] = dummy;
		// }

		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to
									// terminate in order for the user program
									// to terminate
		navThread.start();
	}

	// Inputs

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map) {
		this.map = map;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn) {
		this.parkingSlotDetectionIsOn = isOn;
	}

	// Class control

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#updateNavigation()
	 */

	public synchronized void updateNavigation() {
		this.updateSensors();
		this.calculateLocation();
		if (this.parkingSlotDetectionIsOn) {
			try {
				this.detectParkingSlot();
			} catch (Exception e) {
				Sound.beep();
			}

		}
	}

	// Outputs

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose() {
		return this.pose;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		
		return list_ParkingSlot;
	}
//	/**
//	 * return actual line for Guidance
//	 */
	
	//ins Interface über nehmen //TODO
	
	/**
	 * returns the Line the robot actual drives for guidance
	 */
	public int getLine(){
		int line;
		line=akt_linie;
		return line;
	}
	
	/**
	 * returns the angle of the actual line of the robot for guidance
	 */
	
	public float getPhi(){
		float phi;
		phi= (float) phi_kontroll;
		return phi;
	}

	
	
	
	
	//---------------------------------------------------------------------------------------------------------------
	// Private methods

	/**
	 * calls the perception methods for obtaining actual measurement data and
	 * writes the data into members
	 */
	private void updateSensors() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();

		this.angleMeasurementLeft = this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight = this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement = this.mouseodo.getOdoMeasurement();
		this.mouseTime = this.mouseOdoMeasurement.getDeltaT();
		this.mouseX = this.mouseOdoMeasurement.getUSum();
		this.mouseY = this.mouseOdoMeasurement.getVSum();
		this.mouseX=this.mouseX;     // switch from mm to cm
		this.mouseY=this.mouseY;     // switch from mm to cm

		// values given by PERCEPTION in mm -> here: switch them to cm
		this.frontSensorDistance = perception.getFrontSensorDistance() / 10;
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance() / 10;
		this.backSensorDistance = perception.getBackSensorDistance() / 10;
		this.backSideSensorDistance = perception.getBackSideSensorDistance() / 10;

	}

	/**
	 * detects the robots environnement with the distance-sensors
	 */
	private void ist_irgendwo_was() {
		if ((this.frontSensorDistance < 30) && (this.frontSensorDistance != 0)) {       // Wert an Roboter anpassen
			vorne_ist_was = true;
		} else {
			vorne_ist_was = false;
		}
		if (this.backSensorDistance < 8 && this.backSensorDistance != 0) {
			hinten_ist_was = true;
		} else {
			hinten_ist_was = false;
		}
		if (this.frontSideSensorDistance < 13 && this.backSideSensorDistance < 13 && (this.frontSideSensorDistance != 0) && (this.backSideSensorDistance != 0)) {
			seite_ist_was = true;
		} else {
			seite_ist_was = false;
		}
	}

	/**
	 * detects the robots pose from measurements
	 */
	private void calculateLocation() {

		vorne_ist_was = false;
		seite_ist_was = false;
		hinten_ist_was = false; // initialise environnement
		ist_irgendwo_was();
//		double leftAngleSpeed = this.angleMeasurementLeft.getAngleSum() / ((double) this.angleMeasurementLeft.getDeltaT() / 100000);
//		// degree/seconds
//		double rightAngleSpeed = this.angleMeasurementRight.getAngleSum() / ((double) this.angleMeasurementRight.getDeltaT() / 100000);
//		// degree/seconds
//
//		double vLeft = (leftAngleSpeed * Math.PI * LEFT_WHEEL_RADIUS) / 180;
//		// velocity of left wheel in cm/s
//		double vRight = (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180;
//		// velocity of right wheel in cm/s
//		double w = (vRight - vLeft) / WHEEL_DISTANCE;
//		// angular velocity of robot in rad/s
//
//		double vMid = (vRight + vLeft) / 2;
//		// velocity of the center of the axis
//		double deltaT = ((double) this.angleMeasurementRight.getDeltaT()) / 100000;
		// time-interval
		//
		// //Annahme: vLeft, VRight und deltaT sind w‰hrend des kurzen
		// Zeitintervalls gleich
//		if (vRight - vLeft > 10 || vLeft - vRight > 10) {
//			poseFehler++;
//		}
		
	
//		//außerhalb der Karte mit Maussensor navigieren
//				if((this.pose.getX()>1.89 && akt_linie==1)){
//					mouseNav=true;
////					onLine=false;
//				}
//				if(this.pose.getY()<-0.07 && akt_linie==0){
//					mouseNav=true;
////					onLine=false;
//				}
//				if(this.pose.getY()>0.39 && akt_linie==4){
//					mouseNav=true;
////					onLine=false;
//				}
	
				
				
		//Radodometrie
				double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
				double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds
				double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
				double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
				double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
				Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
				double ICCx 		= 0;
				double ICCy 		= 0;
				double xResult 		= 0;
				double yResult 		= 0;
				double angleResult 	= 0;
				double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;	
//				//außerhalb der Karte mit Maussensor navigieren
//						if((this.pose.getX()>1.89 && akt_linie==1)){
//							mouseNav=true;
////							onLine=false;
//						}
//						if(this.pose.getY()<-0.07 && akt_linie==0){
//							mouseNav=true;
////							onLine=false;
//						}
//						if(this.pose.getY()>0.39 && akt_linie==4){
//							mouseNav=true;
////							onLine=false;
//						}
			
						
						if(parkingactive==false){
							onLine=true;
						}else{
							onLine=false;
						}
				//Radodometrie
					
		//onLine=false;
						// auswertung ermitteln der strecke und des winkels noch drei faellen: steht, gerade fahrt, drehend
						if (R.isNaN()) { //robot don't move
							xResult			= this.pose.getX();
							yResult			= this.pose.getY();
							angleResult 	= this.pose.getHeading();
						} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight
							xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
							yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
							angleResult 	= this.pose.getHeading();
				         }
						else {			
							ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
							ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
						
							xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
							yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
							angleResult 	= this.pose.getHeading() + w * deltaT;
						}
						
				
				
				
				
//		double xResult =e*( this.pose.getX() + vMid * deltaT * Math.cos(this.pose.getHeading() + 0.5 * w * deltaT)); // in cm
//		double yResult =e*( this.pose.getY() + vMid * deltaT * Math.sin(this.pose.getHeading() + 0.5 * w * deltaT)); // in cm
//		double angleResult = (this.pose.getHeading() + w * deltaT)*1;
//		
//		double deltax=0;
//		double deltay=0;
//		
//		deltax= xResult-this.pose.getX();
//		deltay= yResult- this.pose.getY();
//		
//		
//		
//		// if Sensors show same distances -> angleResult = phi_kontroll;
		 // else it will be calculated
		// normally

		// with the actual change of Heading a corner can be detected
		detectCorner();
		// with the line-sensors -> detect a touched line
	//	detectline();
		// if ((this.pose.getHeading() - alterwinkel > -70)
		// && (this.pose.getHeading() != alterwinkel)) {
		// detectionecke = true;
		// rechtskurve = false;
		// linkskurve = true;
		// }
		//
		// }
		// Detektion auf der Linie anhand Abstand zur akt Linie
		 
//		 Point koordinate = new Point(0, 0);
//		 koordinate.setLocation(xResult, yResult);
//		for (int i = 0; i < 8; i++) {
//		abstand = Math.abs(map[i].ptLineDist(koordinate));
//		if (abstand < 5) {
//		akt_linie = i;
//		} else {
//	 // Nothing to do here!
//		//
//		 }
//		 }
		// }
		//reset Pose
		if(onLine==true){
		if(akt_linie==0 || akt_linie==1 || akt_linie==4 || akt_linie==7){
			if(this.pose.getX()==0.60 && akt_linie==0){
				yResult=0;
				angleResult=0;
			}
			if(this.pose.getX()==1.20 && akt_linie==0){
			yResult=0;
			angleResult=0;
			}
			if(this.pose.getY()==0.30 && akt_linie==1){
			xResult=1.80;
			angleResult=90*we;
			}
			if(this.pose.getX()==1.30 && akt_linie==4){
			yResult=0.30;
			angleResult=180*we;
			}
			if(this.pose.getX()==1.40 && akt_linie==4){
				yResult=0.30;
				angleResult=180*we;
				}
				
			if(this.pose.getX()==0.75 && akt_linie==4){
			yResult=0.30;
			angleResult=180*we;
			}
			if(this.pose.getY()==0.30 && akt_linie==7){
			xResult=0;
			angleResult=270*we;
			}
		
		
}
		}
		
		// know on which line the roboter is -> when a corner was detected ->
		// line switch and known points are set as actual pose
	if(onLine==true){
		if (detectionecke == true) {
//			poseFehler=0;
			switch (akt_linie) {
			case 0:
				if(this.pose.getX()>1.6*e){
				xResult = 1.80*e;
				yResult = 0*e;
			//	angleResult= 90*we;
				akt_linie++;
				phi_kontroll = 90*we;
				}
				break;
			case 1:
				if(this.pose.getY()>0.50){
				xResult = 1.80*e;
				yResult = 0.60*e;
			//	angleResult= 180*we;
				akt_linie++;
				phi_kontroll = 180*we;
				}
				break;
			case 2:
				if(this.pose.getX()<1.60){
				xResult = 1.50*e;
				yResult = 0.60*e;
			//	angleResult=270*we;
				akt_linie++;
				phi_kontroll = 270*we;
				}
				break;
			case 3:
				if(this.pose.getY()<0.40){
				xResult = 1.50*e;
				yResult = 0.30*e;
//				angleResult=180*we;
				akt_linie++;
				phi_kontroll = 180*we;
				}
				break;
			case 4:
				if(this.pose.getX()<0.40){
				xResult = 0.30*e;
				yResult = 0.30*e;
			//	angleResult=90*we;
				akt_linie++;
				phi_kontroll = 90*we;
				}
				break;
			case 5:
				if(this.pose.getY()>0.50){
				xResult = 0.30*e;
				yResult = 0.60*e;
				akt_linie++;
//				phi_kontroll = 180*we;
				}
				break;
			case 6:
				if(this.pose.getX()<0.10){
				xResult = 0*e;
				yResult = 0.60*e;
			//	angleResult= 270*we;
				akt_linie++;
				phi_kontroll = 270*we;
				}
				break;
			case 7:
				if(this.pose.getY()<0.1){
				xResult = 0*e;
				yResult = 0*e;
			//	angleResult=0*we;
				akt_linie = 0;
				phi_kontroll = 0*we;
				}
				break;

			default:// no action here

				break;
			}
		}
	}
//		// Navigation by Mouse when measurement of ODO is bad or when vMid is very small
//		// vmid in cm/s

		//angle with Triangulation and reset
//	if(onLine==true)	
//		angleResult = angleWithTriang(w, deltaT);
		
//		mouseNav=false;
////		if ((vMid < 0.05 && mouseNav == true) || (poseFehler > 200 && mouseNav == true)) {
//		if(mouseNav==true){ //FIXME
//			phiMaus= Math.abs(Math.cos(phi_kontroll))*(Math.tan((mouseX/mouseY)))+ Math.abs(Math.sin(phi_kontroll))*Math.tan((mouseY/mouseX));
//			xResult =e*( this.pose.getX() - mouseX*Math.sin(phiMaus)+ mouseY*Math.cos(phiMaus));
//			yResult =e*( this.pose.getY() + mouseY*Math.sin(phiMaus)+ mouseX*Math.cos(phiMaus));
//			angleResult = phiMaus*we;
//			
//		}
		
		
		
//			xResult =e*( this.pose.getX() - 0.1*mouseX*Math.cos(this.pose.getHeading())+mouseY*Math.sin(this.pose.getHeading()));
//			yResult =e*( this.pose.getY() + 0.1*mouseY*Math.sin(this.pose.getHeading())-mouseX*Math.cos(this.pose.getHeading()));
			if(onLine==true){
//			poseFehler=0;
			if(this.pose.getX()<1.5 && this.pose.getX()>0.2 && akt_linie==0) angleResult=phi_kontroll;
			if(akt_linie==7 && this.pose.getY()<0.5 && this.pose.getY()>0.1) angleResult= phi_kontroll;
			if(akt_linie==4 && this.pose.getX()<1 && this.pose.getX()>0.65) angleResult=phi_kontroll;
			if(this.pose.getX()<1.2 && this.pose.getX()>0.9 && akt_linie==4) angleResult=phi_kontroll;
			if(this.pose.getX()<0.8 && this.pose.getX()>0.6 && akt_linie==4) angleResult=phi_kontroll;
			if(this.pose.getY()>0.2 && this.pose.getY()<0.45&& akt_linie==1) angleResult=phi_kontroll;
	//	}	
//			if(seite_ist_was==false && akt_linie==0 && this.pose.getX()<1.6 && this.pose.getX()>0.05) angleResult=0;
//			if(this.frontSensorDistance>50 && akt_linie== 0 && this.pose.getX()<1.2 && this.pose.getX()>0.15){
//				angleResult=phi_kontroll;
//				
//			}
//			if(this.frontSensorDistance>50 && akt_linie==4 && this.pose.getX()>0.6 && this.pose.getX()<1.2){
//				angleResult=phi_kontroll;
//				yResult=0.30;
//			}
//			if(this.backSensorDistance>50 && akt_linie==0 && this.pose.getX()>1.2 && this.pose.getX()<1.6){
//				angleResult=phi_kontroll;
//			}
//			}
			}
		
		// detection ON the line by Line Sensor and no corner is being detected
//		if (detectionline==true && onLine==true) {
//	//		xResult = map[akt_linie].getX1() * Math.abs((Math.sin(phi_kontroll)))+this.pose.getX(); // if line sensor detect a line -> one certain coordinate can be fixed
//																					
//	//		yResult = map[akt_linie].getY1() * Math.abs((Math.cos(phi_kontroll)));
//			switch (akt_linie) {
//			case 0:
//				if(this.pose.getX()<1.70*e && this.pose.getX()>0.07*e){
//				yResult = 0*e;
//			//	angleResult= 0*we;
//				}
//				break;
//			case 1:
//				if(this.pose.getY()<0.50*e && this.pose.getY()>0.07*e){
//				xResult = 1.80*e;
//			//	angleResult= 90*we;
//				}
//				break;
//			case 2:
//				if(this.pose.getX()<1.75*e && this.pose.getX()>1.55*e){
//				yResult = 0.60*e;
//			//	angleResult=180*we;
//				break;
//				}
//			case 3:
//				if(this.pose.getY()<0.5*e && this.pose.getY()>0.4*e && this.pose.getX()<160 && this.pose.getX()>140){
//				xResult = 1.50*e;
//			//	angleResult=270*we;
//				}
//				break;
//			case 4:
//				if(this.pose.getX()<1.40*e && this.pose.getX()>0.5*e){
//				yResult = 0.30*e;
//			//	angleResult=180*we;
//				}
//				break;
//			case 5:
//				if(this.pose.getY()<0.50*e && this.pose.getY()>0.37*e){
//				xResult = 0.30*e;
//			//	angleResult=90*we;
//				}
//				break;
//			case 6:
//				if(this.pose.getX()<0.35*e && this.pose.getX()>0.05*e){
//				yResult = 0.60*e;
//			//	angleResult= 180*we;
//				}
//				break;
//			case 7:
//				if(this.pose.getY()<0.45*e && this.pose.getY()>0.1*e){
//				xResult = 0.0*e;
//			//	angleResult=270*we;
//				}
//				break;
//
//			default:// no action here
//
//				break;
//			}
//			
//		}

		// reset angle each round from 360 degree to 0 degree
		if (angleResult > 2*Math.PI)
			angleResult -= 2*Math.PI;
		if (angleResult < 0)
			angleResult += 2*Math.PI;

		alterwinkel = this.pose.getHeading(); // save angle to compare it in the
												// next step in degree
		reset(); // reset the used variables for steps in navigation
		
		
//		if(parkingactive==true){
//			if(this.backSensorDistance<60) Sound.playTone(10, 1, 1);
//			if(this.backSensorDistance<50) {
//				Sound.playTone(10, 1, 1);
//				Sound.playTone(10, 1, 1);
//			}
//			if(this.backSensorDistance<40){
//				Sound.playTone(10, 10);
//			}
//		}
		
		
		// set pose , given in CM
		this.pose.setLocation((float) xResult, (float) yResult);
		this.pose.setHeading((float) angleResult); // in degree
	}

	/**
	 *  reset the boolean parameters for calculating location
	 */
	private void reset() {
		detectionecke=false;	// reset for next calculation
		detectionline=false;	// reset for next calculation	
	}

	/**
	 * get the actual state of the roboter from guidance for improving navigation or set it as the navigation out of the parcours
	 */
	public void setParkingActive(boolean state){
		parkingactive=state;
	}
	
	/**
	 * detects a line with the 2 line sensors and the actual line -> coordinates could be set
	 */
	private void detectline() { //FIXME geändert von 2 auf 100 auf 80
		if((this.lineSensorLeft==2) || (this.lineSensorRight==2)){
			detectionline=true;
		}else{
			detectionline=false;
		}
	}

	
	/**
	 * detects a corner with the actual heading in comparison with the last one (70 degree difference)
	 */
	private void detectCorner() {
		
		if ((Math.abs(this.pose.getHeading() - phi_kontroll) > Math.PI*0.3) && Math.abs(this.pose.getHeading()-phi_kontroll)< Math.PI*0.6) {
			detectionecke = true;
			rechtskurve = true;
			linkskurve = false;
		}else{
			detectionecke=false;
		}
	}
	
	
	

	/**
	 * calculates the angle result with the triangulation sensors -> if their distance is nearly the same one
	 * @param w
	 * @param deltaT
	 * @return
	 */
	private double angleWithTriang(double w, double deltaT) {
		double angleResult;
		if ((seite_ist_was == true) && (Math.abs(this.frontSideSensorDistance - this.backSideSensorDistance) <4)) {
			angleResult = phi_kontroll*Math.PI/180;
		}else{
			angleResult=(this.pose.getHeading() + w * deltaT);
		}
		return angleResult;
	}

	
	

	/**
	 * detects parking slots and manage them by initializing new slots,
	 * re-characterizing old slots or merge old and detected slots. tats‰chliche
	 * Werte der Lücke gespeichert -> nicht nur Pose des Roboters
	 * 
	 * @throws Exception
	 */
	
	// Exception is possible -> when array is full and no more ParkingSlots can
	// be detected!
	
	int frontY=7; //in cm //TODO Werte eintragen
	int frontX=0; //in cm
	int backY=5; // in cm
	int backX=0; // in cm
	
	private void detectParkingSlot() throws Exception {
	//	ParkingSlotStatus slotStatus = ParkingSlotStatus.BAD; // first, set
																// SlotStatus to
																// BAD
	//	double groesse = 0; // Size of the Slot in cm

		double y_abst_backsidesens = 3; // cm
		double x_abst_frontsidesens = 5; // cm
		double y_abst_frontsidesens = 3; // cm
		double x_abst_backsidesens = 0; // cm
		double korrektur1 = Math.sin(phi_kontroll) * y_abst_backsidesens - Math.cos(phi_kontroll) * x_abst_backsidesens;
		double korrektur2 = Math.sin(phi_kontroll) * x_abst_backsidesens - Math.cos(phi_kontroll) * y_abst_backsidesens;
		double distancefront = 0;
		double distanceback = 0;
//		double groesse_neu = 0;
		// double
		// korrektur3=Math.sin(this.pose.getHeading())*this.backSideSensorDistance
		// - Math.cos(this.pose.getHeading())*this.backSideSensorDistance;
		// double
		// korrektur4=Math.cos(this.pose.getHeading())*this.backSideSensorDistance
		// - Math.sin(this.pose.getHeading())*this.backSideSensorDistance;

		//gueteSlot();
		// check guete before the measurement of the slot

// ------------------MAIN------------------------------------------------------------------------
	if(parkingactive==true){
		suchen=false;
	}else{
		suchen= true;
	}
		
	if(akt_linie==2 || akt_linie== 3 || akt_linie==5 || akt_linie== 6 || akt_linie == 7){
		detectionactive=false;
		speichern_bereit=false;
		suchen=false;
	}
	if(this.pose.getY()<0.5 && akt_linie==7){
		detectionactive= false;
		speichern_bereit=false;
	}
	
	if(this.pose.getX()>1.7 && akt_linie==0){
		detectionactive=false;
		speichern_bereit=false;
	}
	if((akt_linie==2 || akt_linie==1)&&(parkingactive==true)) akt_linie=1;	
		
		
		if(suchen==true){	
		if(akt_linie==0 || akt_linie==1 || (akt_linie==4 /* && this.pose.getX()<1.3 && this.pose.getX()>0.45) */)){		//only on line 0,1,4 search for slots
			
			bmeasurement(); // Fehlerfälle ausschließen
	//		bmeasurement=true;
			gueteSlot=true;
			
			
	//	bmeasurement=true;
			if (this.gueteSlot == true) {
			
						if(speichern_bereit==false && detectionactive==false){
							checkFront();
							}
						if (detectionactive == true&&speichern_bereit==false) {
								if(idWinkel<200){
									listWinkel[idWinkel] = this.pose.getHeading(); // save Angle
//																 from actual
//																 Point to get
//																 a judgement
//																 of the
//																 measurement
									idWinkel++;					//
									checkBack();				//
								}else{									// shows the next angle to save
									checkBack();							// check backboundary
								}
						}
			// checkGuete(); // Guete is guessed

			if (speichern_bereit == true && detectionactive==false) {
					average(); // calculates the average of the angles during the
							// measurement -> weniger als 2 Grad Abweichung
						if (Math.abs(average- 90*we)  > 10*Math.PI/180 && akt_linie==1) {
							gueteMeasurement = false;
					
//							slotStatus=ParkingSlotStatus.RESCAN;
//					 BAD measurement -> Slot will be safed as RESCAN
						} else if(Math.abs(average- 180*we) > 10*Math.PI/180 && akt_linie==4){
							gueteMeasurement= false;
						} else if(akt_linie==0){
							gueteMeasurement = true;
						}
						
						else {
							gueteMeasurement = true; // Measurement was good
						}
//			gueteMeasurement=true;
//				if (gueteMeasurement == true || gueteMeasurement==false){
					
							checkIdentity(); // check if Slot already exists
							save();
							}
//					}
//					gueteSlot = false; // initialise measurement of Guete
			} else {
			// nothing to do when Measurement will be BAD
					//LCD.drawString("Guete schlecht", 0, 5); // write on LCD if
													// measurement of Slot will
													// be bad and not measured
				gueteSlot=false;
		}
		}else{
			//LCD.drawString("keine Slots auf der Linie", 0, 5);
		}
// --------------------------------------------------------------------------------------------------------------

		// if (bekannt=true){
		// distancefront=list_ParkingSlot[id_bekannte_luecke].getFrontBoundaryPosition().distance(anfang);
		// distanceback=
		// list_ParkingSlot[id_bekannte_luecke].getBackBoundaryPosition().distance(ende);
		//
		// distancefront = Math.round(distancefront*1000)/10000.0;
		// distanceback= Math.round(distanceback*1000)/10000.0;

		// if (distancefront<1 && distanceback<1){
		// // so lassen
		// }else{
		// neu einspeichern
		// }
		// }

	}	
	}
	

	/**
	 * filtert Fehlerfälle heraus
	 */
	private void bmeasurement() {
		if((this.pose.getX()>1.45*e && akt_linie==4)|| (this.pose.getX()>1.80*e && akt_linie==0) || (this.pose.getY()>0.6*e && akt_linie==1) || (this.pose.getX()<0.03*e && akt_linie==0)
				|| (this.pose.getY()<-0.1*e && akt_linie==1) ||(this.pose.getX()<0.35 && akt_linie==4)){
			bmeasurement=false;
		}else{
			bmeasurement=true;
		}
	}

	/**
	 * bildet Mittelwert der Winkel während der Messung
	 */
	private void average() {
		for (int count = 0; count > idWinkel; count++) {
			summe = summe + listWinkel[idWinkel];

		}
		average = summe / idWinkel;
	}

	/**
	 * save the slot -> throw exception if the array is full and no more slots can be detected
	 * @throws Exception
	 */
	private void save() throws Exception {
		boolean wahr = false;
		
		double groesse=0.0;
//		ParkingSlot Luecke2 = new ParkingSlot(0, null, null, null);
		// judge the slotLength
//		gueteMeasurement=true;
//		if(gueteMeasurement==true){
//		slotStatus = checkLength();
//		}else{
//			slotStatus=ParkingSlotStatus.RESCAN;
////			Luecke2.setStatus(slotStatus);
//		}
		if(akt_linie==0 && anfang.getX()<1.70 && anfang.getX()>0.04 && ende.getX()<1.75 && ende.getX()>0.05){
			wahr=true;
		}
		if(akt_linie==1 /*&& anfang.getY()<0.6 && anfang.getY()>0.00 && ende.getY()<0.6 && ende.getY()>0.05 &&anfang.getX()>1.90 && ende.getX()>1.90*/){
			wahr=true;
		}
		if(akt_linie==4 && anfang.getX()<1.3 && anfang.getX()>0.45 && ende.getX()<1.20 && ende.getX()>0.4){
			wahr = true;
		}
		
		if(anfang.getX()==2.10 && ende.getX()==2.10) wahr=true;
		if(anfang.getY()==-0.30 && ende.getY()==-0.30) wahr= true;
//		if(anfang.getY()==0.60 && ende.getY()==0.60) wahr = true;
			
		
			if(akt_linie==0){
			groesse= Math.abs(ende.getX()-anfang.getX());
//			groesse= Math.abs(anfang.distance(ende));
			}else if(this.pose.getX()>1.65 && this.pose.getY()>0.15 && this.pose.getY()<0.65){
			groesse=Math.abs(ende.getY()-anfang.getY());	
//			groesse = Math.abs(anfang.distance(ende));
			}else if(akt_linie==4){
			groesse=Math.abs(anfang.getX()-ende.getX());	
			}
		
		
//		if (id_aktuell < 25) {
			
			if(bekannt==false && wahr==true){
//			list_ParkingSlot[id_aktuell] = new ParkingSlot(id_aktuell, anfang, null, null);		//TODO check how it works
			
//				slotStatus=checkLength();
			ParkingSlot Luecke = new ParkingSlot(0, null, null, null);
//			Sound.beep();
			Luecke.setBackBoundaryPosition(ende);
			Luecke.setFrontBoundaryPosition(anfang);
			Luecke.setID(id_aktuell);
			if(groesse>=0.47 /*&& gueteMeasurement == true*/){
				
				// for TEST to Rescan //TODO
			Luecke.setParkingSlotStatus(ParkingSlotStatus.GOOD);}
			else if(groesse<=0.45 /*&& gueteMeasurement==true*/){
				Luecke.setParkingSlotStatus(ParkingSlotStatus.BAD);	
			}
			else if((groesse<0.47 && groesse >0.45) /*|| gueteMeasurement==false*/) Luecke.setParkingSlotStatus(ParkingSlotStatus.RESCAN);
			else Luecke.setParkingSlotStatus(ParkingSlotStatus.RESCAN);
//			slotStatus= checkLength();
//			Luecke2.setStatus(slotStatus);
			list_ParkingSlot[id_aktuell]=Luecke;
			//list_ParkingSlot[id_aktuell] =Luecke;
//			list_ParkingSlot[id_aktuell].setFrontBoundaryPosition(anfang);
//			list_ParkingSlot[id_aktuell].setBackBoundaryPosition(ende);
//			list_ParkingSlot[id_aktuell].setID(id_aktuell);
//			list_ParkingSlot[id_aktuell].setParkingSlotStatus(slotStatus);
//			list_ParkingSlot[id_aktuell].setFrontBoundaryPosition(anfang);
//			list_ParkingSlot[id_aktuell].setBackBoundaryPosition(ende);
//			list_ParkingSlot[id_aktuell].setStatus(slotStatus);
			
			//reinitialising
			id_aktuell++; // pointer to next slot in the array
			speichern_bereit = false;
			bmeasurement=false;
			detectionactive = false;
			bekannt = false;
			gueteMeasurement= false;
			gueteSlot=false;
			idWinkel=0;
			wahr=false;
//			Sound.playTone(5, 10, 1);
//			Sound.beep();
						}else{
							if(list_ParkingSlot[id_bekannte_luecke].getStatus().ordinal()==ParkingSlotStatus.RESCAN.ordinal() && this.gueteMeasurement==true){
								list_ParkingSlot[id_bekannte_luecke].setFrontBoundaryPosition(anfang);
								list_ParkingSlot[id_bekannte_luecke].setBackBoundaryPosition(ende);
								if(groesse>47)
								list_ParkingSlot[id_bekannte_luecke].setParkingSlotStatus(ParkingSlotStatus.GOOD);
								if(groesse<45)
									list_ParkingSlot[id_bekannte_luecke].setParkingSlotStatus(ParkingSlotStatus.BAD);
								
								
							}
						}
//				if(bekannt=true){
//					//nothing to do here!
//				}
			//evtl ARRAY der Winkel listWinkel löschen 
//		} else {
//			throw new Exception("No space for slots"); // Exception thrown if array is
												// full
//		}

	}

////	/**
////	 * @return
////	 */
////	private ParkingSlotStatus checkLength() {
////		ParkingSlotStatus slotStatus;
////		double groesse;
////		groesse =0.0;
////		//groesse = Math.abs(anfang.distance(ende)); // in m
////		if(akt_linie==0){
////		groesse=ende.getX()-anfang.getX();
////		}else if(akt_linie==1){
////		groesse=ende.getY()-anfang.getY();	
////		}else if(akt_linie==4){
////		groesse=anfang.getX()-anfang.getY();	
////		}else{
////		//nothing
////		}
////		// TODO Werte aufnehmen in cm 45cm
////		if (Math.abs(groesse) < 0.45*e) {
////			// too small for robot -> switch state to BAD
////			slotStatus = ParkingSlotStatus.BAD;
//////		} else if ((groesse < 30) && (groesse > 28.1)) {
//////			// switch state to RESCAN because it is in a critical intervall
//////			slotStatus = ParkingSlotStatus.RESCAN;
////		} else {
////			// switch state to GOOD
////			slotStatus = ParkingSlotStatus.GOOD;
////FIXME
//		}
//		return slotStatus;
//	}

	/**
	 * checks if the backboundary of the slot is detected
	 */
	private void checkBack() {
//		if(akt_linie==1 && this.pose.getY()>50) abstand_sens_band=25;
//		else abstand_sens_band=25;
		
		//detection on line 0
		if (akt_linie==0 &&(this.frontSideSensorDistance < 22) && detectionactive == true && bmeasurement==true && this.frontSideSensorDistance!=0 && this.backSideSensorDistance>25) {

			ende= new Point (((float)(this.pose.getX()+0.01*(frontY+3))), (float) (-0.3));
			
//			endpos=new Point((float) this.pose.getX(), (float) this.pose.getY());
			detectionactive=false;
			speichern_bereit = true; // save Slot NOW!
			
		}
		// detection on line 1
		if (akt_linie==1 &&(this.frontSideSensorDistance < 25) && this.backSideSensorDistance>20 && detectionactive == true && bmeasurement==true && this.frontSideSensorDistance!=0) {

			ende= new Point (((float)(0.01*210 )), (float) (this.pose.getY()+0.075));
			
//			endpos=new Point((float) this.pose.getX(), (float) this.pose.getY());
			detectionactive=false;
			speichern_bereit = true; // save Slot NOW!
			
		}
		
		//detection on line 4
		if(akt_linie==4 && this.pose.getX()>0.4 && this.pose.getX()<1.0){
			
			if(this.frontSideSensorDistance<30 && this.backSideSensorDistance>45 && bmeasurement==true && this.frontSideSensorDistance!=0){
				ende= new Point (((float)(this.pose.getX() -0.01*(frontY+3))), (float) (0.60));
//				Sound.playTone(10, 30, 1);
//				ende= new Point ((float) 0.7,(float) 0.6);
			detectionactive=false;
			speichern_bereit=true;
			
			}
			
		}
	}

	/**
	 * checks if the front boundary of the slot is detected and save the local point with the correction according to the actual line
	 */
	private void checkFront() {
//		if(akt_linie==0) abstand_sens_band=25;
//		else if(akt_linie==1) abstand_sens_band=19;
//		else if(akt_linie==4) abstand_sens_band=25;
//		else abstand_sens_band=25;
		
		
		//detection on Line 0
		if ((akt_linie==0 && this.frontSideSensorDistance > abstand_sens_band) && (this.backSideSensorDistance>abstand_sens_band) && (this.frontSideSensorDistance != 0) && (this.backSideSensorDistance != 0) && (detectionactive == false) && (bmeasurement=true)) {			
//			
			anfang=new Point((float) (this.pose.getX()-  0.14), (float) (-0.01*30) ); 
			detectionactive = true;
			beginpos= new Point((float) this.pose.getX(), (float) this.pose.getY());
			
		
			// ready to measure second boundary
		}
		
		//detection on line 1
		if ((akt_linie==1 && this.backSideSensorDistance > 27) && this.frontSideSensorDistance>25 && (this.frontSideSensorDistance != 0) && (this.backSideSensorDistance != 0) && (detectionactive == false) && (bmeasurement=true) && this.pose.getY()<0.4) {			
//			
			anfang=new Point((float) (0.01*210 ), (float) (this.pose.getY()-0.15) ); 
			detectionactive = true;
			beginpos= new Point((float) this.pose.getX(), (float) this.pose.getY());
			
		
			// ready to measure second boundary
		}
	
		// detection on Line 4
		if(akt_linie==4 && this.pose.getX()<1.4 && this.pose.getX()>0.6){
			if(this.frontSideSensorDistance>40 && detectionactive==false && this.frontSideSensorDistance!=0){
			anfang=new Point(((float) (this.pose.getX() - 0.08)), (float) (0.60) ); 
//				anfang=new Point ((float)1.2,(float)0.6);
			detectionactive=true;
//			Sound.playTone(10, 10, 1);
			}
		}
	
	}
	

	/**
	 * checks if the slot is in the arry yet
	 */
	private void checkIdentity() {

		if (id_aktuell != 0) {
			boolean stop=false;
			double deltax=0.17;
			double deltay=0.17;
			double deltax2=0.17;
			double deltay2=0.17;
			
			for (int i = 0; i < id_aktuell; i++) {
				if(akt_linie==0){
					deltax=(ende.getX())-list_ParkingSlot[i].getBackBoundaryPosition().getX();
					deltay=(ende.getY())-list_ParkingSlot[i].getBackBoundaryPosition().getY();
					deltax2=(anfang.getX())-list_ParkingSlot[i].getFrontBoundaryPosition().getX();
					deltay2=(anfang.getY())-list_ParkingSlot[i].getFrontBoundaryPosition().getY();
				} else if(akt_linie==1){
					deltax=(ende.getX() -list_ParkingSlot[i].getBackBoundaryPosition().getX());
					deltay=(ende.getY())-list_ParkingSlot[i].getBackBoundaryPosition().getY();
					deltax2=(anfang.getX() -list_ParkingSlot[i].getFrontBoundaryPosition().getX());
					deltay2=(anfang.getY())-list_ParkingSlot[i].getFrontBoundaryPosition().getY();
				}
				else	if(akt_linie==4){
					deltax=(ende.getX()) -list_ParkingSlot[i].getBackBoundaryPosition().getX();
					deltay=(ende.getY()) -list_ParkingSlot[i].getBackBoundaryPosition().getY();
					deltax2=(anfang.getX()) -list_ParkingSlot[i].getFrontBoundaryPosition().getX();
					deltay2=(anfang.getY()) -list_ParkingSlot[i].getFrontBoundaryPosition().getY();
				}
				if(stop==true){
//					Sound.beep();
					stop=false;
					break;
					
				}
			
//				double deltax =100*this.pose.getX() + Math.sin(phi_kontroll) * (15)+ Math.cos(phi_kontroll)*frontY - list_ParkingSlot[i].getBackBoundaryPosition().getX(); // IN
				// CM
//				double deltay =100*this.pose.getY() - Math.cos(phi_kontroll) * (15) +Math.sin(phi_kontroll)*frontY- list_ParkingSlot[i].getBackBoundaryPosition().getY(); // IN
					
				
				// CM
				//Grenzwert 5 cm
				if ((Math.abs(deltax) < 0.15) && (Math.abs(deltay) < 0.15)  && (Math.abs(deltax2) < 0.15) && (Math.abs(deltay2) < 0.15)) {
//					if(bekannt==true) break;
					bekannt = true;
					if(bekannt==true) stop=true;
					id_bekannte_luecke = i;
				}
			}
					//überschreibe den Slot wenn die Messung jetzt gut war und der Slot vorher als RESCAN bezeichnet wurde
//					if(list_ParkingSlot[id_bekannte_luecke].getStatus()==ParkingSlotStatus.RESCAN && bekannt==true && gueteMeasurement==true){
//						ParkingSlot Luecke= new ParkingSlot(id_bekannte_luecke, null, null,null);
////						list_ParkingSlot[id_bekannte_luecke]=Luecke;
//						Luecke.setBackBoundaryPosition(ende);
//						Luecke.setFrontBoundaryPosition(anfang);
//						Luecke.setID(id_bekannte_luecke);
//						ParkingSlotStatus slotStatus= checkLength();
//						Luecke.setParkingSlotStatus(slotStatus);
//						Sound.beep();
////						list_ParkingSlot[id_bekannte_luecke].setFrontBoundaryPosition(anfang);
////						list_ParkingSlot[id_bekannte_luecke].setBackBoundaryPosition(ende);
////						list_ParkingSlot[id_bekannte_luecke].setID(id_bekannte_luecke);
////						list_ParkingSlot[id_bekannte_luecke].setParkingSlotStatus(checkLength());
//					}else{
//					id_bekannte_luecke=0; // reset id_bekannte_luecke
////					LCD.("Luecke gefunden", 0, 5);
//					}

				
			
		}

	}

//	/**
//	 * judge the measurement before begin, with the actual heading ( 25 degree )
//	 *
//	 */
//	private void gueteSlot() {
//		double winkeldiff = Math.abs(this.pose.getHeading() -phi_kontroll);
//		if ((winkeldiff > 25*Math.PI/180)&&(winkeldiff<100*Math.PI/180)) {
//			gueteSlot = false;
//		} else {
//			gueteSlot = true;
//		}
//	}

	/**
	 * 
	 * @param i
	 *            ID of the certain SLOT
	 * @return ParkingSlot from the ARRAY with ID for HMI
	 */
	public ParkingSlot getSlotById(int i) {
		return list_ParkingSlot[i];
	}
	
	

}
