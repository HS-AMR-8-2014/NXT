package parkingRobot.hsamr8;

import lejos.geom.Line;
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
	static final double WHEEL_DISTANCE = 0.114; // only rough guess, to be //TODO Messen
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
	boolean parkingSlotDetectionIsOn = false;
	/**
	 * pose class containing bundled current X and Y location and corresponding
	 * heading angle phi
	 */
	Pose pose = new Pose();
	int id_aktuell = 0;
	boolean x_fix = false;
	boolean y_fix = true;
	int fix_value = 0;
	ParkingSlot[] list_ParkingSlot;
	int abstand_sens_band = 16; // Abstand von Sensor zur Bande in mm
	int phi_kontroll = 0;
	int akt_linie = 0;
	int last_linie = 0;
	Double anstieg= new Double (0);
	boolean fixpunktverfahren= false;
	float winkelaenderung=0;
	float zeitaenderung =0;
	double kritischerwegvorn=12; //cm
	boolean detectionecke=false;
	double last_winkel=0;
	double richtung=0;
	double xGenau=0;
	double yGenau=0;
	int minimalabstand=11;
	boolean seite_ist_was=false;
	boolean hinten_ist_was=false;
	boolean vorne_ist_was=false;
	boolean detectionactive = false;
	int abstandtriang=4;
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
		if (this.parkingSlotDetectionIsOn)
			this.detectParkingSlot();
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
		return null;
	}

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

		this.frontSensorDistance = perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance = perception.getBackSensorDistance();
		this.backSideSensorDistance = perception.getBackSideSensorDistance();
		/**AngleDifferenceMeasurement rightEncoder = perception.getControlRightEncoder().getEncoderMeasurement();    //mit ludwig absprechen an welcher stelle zu platzieren
		AngleDifferenceMeasurement leftEncoder = perception.getControlLeftEncoder().getEncoderMeasurement();      //mit ludwig absprechen an welcher stelle zu platzieren
		*/

	}
	
	//Encoder Winkle + Zeit für Control freigeben
	public double getRightEncoderAngle(){
		return (this.angleMeasurementRight.getAngleSum());
	}
	public double getRightEncoderTime(){
		return (this.angleMeasurementRight.getDeltaT());
	}
	public double getLeftEncoderAngle(){
		return (this.angleMeasurementLeft.getAngleSum());
	}
	public double getLeftEncoderTime(){
		return (this.angleMeasurementLeft.getDeltaT());
	}
	

	/**
	 * NAV errechnet aktuelle Linie auf der sich der Roboter befindet
	 * 
	 * @param enabled
	 *            ist die Berechnung aktiviert?
	 * 
	 */

	public void update_nav_line(boolean enabled) {
	
		if(enabled){
			
		
		int i=0;
		int winkel2=0;
		float winkel=this.getPose().getHeading();
		
		if(winkel>75 && winkel<105){
			winkel2=90;
		}else if(winkel>165 && winkel<195){
			winkel2=180;
		}else if(winkel>0 && winkel<15){
			winkel2 = 0;
		}else if(winkel>255 && winkel<285){
			winkel2=270;
		}
		
		if(this.frontSensorDistance<14){
			vorne_ist_was=true;
		}else{
			vorne_ist_was=false;
		}
		if(this.frontSideSensorDistance<minimalabstand && this.backSideSensorDistance<minimalabstand){
			seite_ist_was=true;
		}else{
			seite_ist_was=false;
		}
		if(this.backSensorDistance<9){
			hinten_ist_was=true;
		}else{
			hinten_ist_was=false;
		}
		
		
		
		if(vorne_ist_was==true){
			switch (winkel2) {
			case 0:
				phi_kontroll = 90; //Winkel der nächsten Geraden, sozusagen Winkel nach Drehung
				last_linie=akt_linie; //Die akt. linie wird zu nächsten linie
				akt_linie = 1; //setzen der neuen linie
				xGenau=180; //fixpunkt x
				yGenau=0;	//fixpunkt y
				detectionecke=true; //Ecke wurde detektiert
				break;
			case 90:
				
				phi_kontroll = 180;
				detectionecke=true;
				last_linie=akt_linie;
				akt_linie = 2;
				xGenau=180;
				yGenau=60;
				
				break;
			case 180:
				detectionecke=true;
				
				if(last_linie==1){
				last_linie=akt_linie;
				akt_linie = 2;	
				phi_kontroll = 270;
				xGenau=150;
				yGenau=60;		
				}else{
					last_linie=akt_linie;
					akt_linie=6;
					phi_kontroll = 270;
					xGenau=0;
					yGenau=60;
				}
				
				
				break;
			case 270:
				phi_kontroll = 0;
				last_linie=akt_linie;
				detectionecke=true;
				akt_linie = 0;
				xGenau=0;
				yGenau=0;
				break;
			
			default:// no action here
				
				break;
			}
			
		}else if(vorne_ist_was=false){
			switch (winkel2) {
			case 0:
				
				break;
			case 90:
				
				break;
			case 180:
				if(this.frontSensorDistance<30){
				phi_kontroll = 90;
				detectionecke=true;
				last_linie=akt_linie;
				akt_linie = 5;
				xGenau=30;
				yGenau=30;
				}
				
				break;
			case 270:
				phi_kontroll = 180;
				detectionecke=true;
				last_linie=akt_linie;
				akt_linie = 4;
				xGenau=150;
				yGenau=30;
				break;
			
			default:// no action here
				
				break;
			}
		}
		
		}
		
		
	}

	/**
	 * calculates the robot pose from the measurements
	 */
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation(){
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
		double abstand_von_bande=0;
		
		
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		//detektieren wo etwas ist
		if(this.frontSensorDistance<11){
			vorne_ist_was=true;
		}
		if(this.backSensorDistance<8){
			hinten_ist_was=true;
		}
		if(this.frontSideSensorDistance<13 && this.backSideSensorDistance<13){
			seite_ist_was=true;
		}
		
		
		
		
		
		
		if (R.isNaN()) { //robot don't move
			xResult			= this.pose.getX();
			yResult			= this.pose.getY();
			angleResult 	= this.pose.getHeading();
			
			
		} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight
			if(seite_ist_was==true){
				abstand_von_bande=(this.frontSideSensorDistance+this.backSideSensorDistance)/2;
				xResult= this.pose.getX()+ 0.5*(vLeft+vRight)*Math.tan((this.frontSideSensorDistance-this.backSideSensorDistance)/abstandtriang)*deltaT;
				yResult= this.pose.getY()+ 0.5*(vLeft+vRight)*Math.tan((this.frontSideSensorDistance-this.backSideSensorDistance)/abstandtriang)*deltaT;
				angleResult= Math.tan((this.frontSideSensorDistance-this.backSideSensorDistance)/abstandtriang) ;
			}else{
			
			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			angleResult 	= this.pose.getHeading();
			}
			
			
		} else {
			

			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			
			
			if(detectionecke=true){
				akt_linie=last_linie;
				
			}
			
			if(this.lineSensorLeft==2 || this.lineSensorRight==2){
			
				if(this.pose.getX()<190 && this.pose.getY()<10){
					yResult=0;	
					xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
				}else if(this.pose.getX()<190 && this.pose.getX()>170 && this.pose.getY()<60){
					xResult=160;
					yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
					//fixpunkt y
				}else if(this.pose.getX()<185 && this.pose.getX()>145 && this.pose.getY()>55){
					yResult=60;	//fixpunkt y
					xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
				}else if(this.pose.getX()<160 && this.pose.getX()>140 && this.pose.getY()>25 &&this.pose.getY()<35){
					xResult=150;	//fixpunkt y
					yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
				}else if(this.pose.getX()<145 && this.pose.getX()>30 && this.pose.getY()>25 &&this.pose.getY()<35){
					yResult=30;	//fixpunkt y
					xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
				}else if(this.pose.getX()<35 && this.pose.getX()>25 && this.pose.getY()>25 &&this.pose.getY()<65){
					xResult=30;	//fixpunkt y
					yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
				}else if(this.pose.getX()<30 && this.pose.getX()>0 && this.pose.getY()>55 &&this.pose.getY()<65){
					yResult=60;	//fixpunkt y
					xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
				}else if(this.pose.getX()<10 && this.pose.getX()>0 && this.pose.getY()>-10 &&this.pose.getY()<60){
					xResult=0;	//fixpunkt y
					yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
				}
					
				
			}else{
			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult 	= this.pose.getHeading() + w * deltaT;
			}
		}
		
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);		 
	}
	/**
	 * detects parking slots and manage them by initializing new slots,
	 * re-characterizing old slots or merge old and detected slots. tatsächliche
	 * Werte der Lücke gespeichert -> nicht nur Pose des Roboters
	 */
	
	
	
	
	
	
	private void detectParkingSlot() {
		//Initialisierung
		//	double deltaT = ((double) this.angleMeasurementLeft.getDeltaT()) / 1000;
		
		Point anfang = new Point(0, 0);
		Point ende = new Point(0, 0);
		boolean bekannt = false; // ist die Lücke bekannt?
		int id_bekannte_luecke = 0; // Nummer der Bek. Lücke
		ParkingSlotStatus slotStatus = ParkingSlotStatus.BAD; // SlotStatus am
																// Anfang setzen auf BAD
		double groesse = 0; // Größe der Lücke in cm
		// später 2-fache Messung und Mittelung implementieren

		
		double x_abst_backsidesens = 50; // Abstand der Sensoren vom Punkt der
											// POSE (X,Y)
		double y_abst_backsidesens = 3; //cm
		double x_abst_frontsidesens = 5; //cm
		double y_abst_frontsidesens = 3; //cm
		// Korrektur der Lücke auf wahren Wert um Drehung durch Phi in
		// Abhängigkeit der Linie
		// phi_kontroll in Abhängigkeit der aktuellen Linie
		double korrektur1 = Math.sin(phi_kontroll) * y_abst_backsidesens
				- Math.cos(phi_kontroll) * x_abst_backsidesens;
		double korrektur2 = Math.sin(phi_kontroll) * x_abst_backsidesens
				- Math.cos(phi_kontroll) * y_abst_backsidesens;
		double distancefront=0;
		double distanceback=0;
		double groesse_neu=0;
		
		// double
		// korrektur3=Math.sin(this.pose.getHeading())*this.backSideSensorDistance
		// - Math.cos(this.pose.getHeading())*this.backSideSensorDistance;
		// double
		// korrektur4=Math.cos(this.pose.getHeading())*this.backSideSensorDistance
		// - Math.sin(this.pose.getHeading())*this.backSideSensorDistance;

		if (this.frontSideSensorDistance > abstand_sens_band // und Winkel
																// gerade später implementieren
				&& this.backSideSensorDistance > abstand_sens_band) {

			if (this.backSideSensorDistance > abstand_sens_band
					&& (detectionactive == false)) {

				// pruefe, ob Parkluecke schon vorhanden
				// deltax und deltay gegebenenfalls nochmal anpassen
				for (int i = 0; i == list_ParkingSlot.length; i++) {
					double deltax = this.pose.getX()
							- list_ParkingSlot[i].getFrontBoundaryPosition()
									.getX(); // IN CM
					double deltay = this.pose.getY()
							- list_ParkingSlot[i].getFrontBoundaryPosition()
									.getY(); // IN CM
					if ((deltax < 5) && (deltay < 5)) {
						bekannt = true;
						id_bekannte_luecke = i;
						// Parklücken später direkt neu vermessen an dieser
						// Stelle und mit alten Daten abgleichen
		
						
					} else {
						anfang.setLocation(this.pose.getX() + korrektur1 + Math.sin(phi_kontroll)*(this.frontSideSensorDistance+this.backSideSensorDistance)/2,
								this.pose.getY() + korrektur2 - Math.cos(phi_kontroll)*(this.frontSideSensorDistance+this.backSideSensorDistance)/2);
						detectionactive = true;

						// Lücke vermessen und Speichern (in cm)
					}
				}

				if (this.backSideSensorDistance < abstand_sens_band
						&& detectionactive == true) {
					ende.setLocation(this.pose.getX() + korrektur1 + Math.sin(phi_kontroll)*(this.frontSideSensorDistance+this.backSideSensorDistance)/2,
							this.pose.getY() + korrektur2 - Math.cos(phi_kontroll)*(this.frontSideSensorDistance+this.backSideSensorDistance)/2);
					detectionactive = false;
					
					//Vergleich mit bekannter Lücke für evtl. Neuvermessung
				//	if (bekannt=true){
				//		distancefront=list_ParkingSlot[id_bekannte_luecke].getFrontBoundaryPosition().distance(anfang);
					//	distanceback= list_ParkingSlot[id_bekannte_luecke].getBackBoundaryPosition().distance(ende);
						//
				//		distancefront = Math.round(distancefront*1000)/10000.0;
				//		distanceback= Math.round(distanceback*1000)/10000.0;
						
				//		if (distancefront<1 && distanceback<1){
				//			// so lassen
				//		}else{
							// neu einspeichern
				//		}
				//	}
					
					
					// Bewertung
					groesse = anfang.distance(ende);
//TODO Werte aufnehmen
					if (groesse < 28) {
						// Status ZU KLEIN BAD
						slotStatus = ParkingSlotStatus.BAD;
					} else if ((groesse < 30) && (groesse >28.1 )) {
						// Status auf ungeau -> neu zu vermessen setzen ->
						// kritischer Wert
						slotStatus = ParkingSlotStatus.RESCAN;
					} else {
						// Status auf GUT setzen
						slotStatus = ParkingSlotStatus.GOOD;

					}

				

					list_ParkingSlot[id_aktuell] = new ParkingSlot(id_aktuell,
							anfang, ende, slotStatus);

					id_aktuell++;
				}

			}

		} else {
			// NICHTS machen wenn keine Lücke detektiert
		}

		// muss abgesichert werden, dass nur positiv/gut vermessene Parklücken
		// abgespeichert werden

		return;
	}

	/**
	 * 
	 * @param i
	 *            ID der gewünschten Parklücke
	 * @return ParkingSlot aus dem ARRAY mit der ID für die anderen Module
	 */
	public ParkingSlot getSlotById(int i) {
		return list_ParkingSlot[i];
	}

}
