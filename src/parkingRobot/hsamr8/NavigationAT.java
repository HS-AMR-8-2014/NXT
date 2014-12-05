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
	static final double WHEEL_DISTANCE = 0.135; // only rough guess, to be
												// //TODO Messen
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
	boolean x_fix = false;
	boolean y_fix = true;
	int fix_value = 0;
	Point dummyPoint=new Point(0,0);
	ParkingSlot[] list_ParkingSlot = new ParkingSlot[25];
	ParkingSlot dummy= new ParkingSlot (0,dummyPoint,dummyPoint,ParkingSlotStatus.DUMMY);
	
	int abstand_sens_band = 20; // Abstand von Sensor zur Bande
	int phi_kontroll = 0;
	int akt_linie = 0;
	int last_linie = 0;
	Double anstieg = new Double(0);
	boolean fixpunktverfahren = false;
	float winkelaenderung = 0;
	float zeitaenderung = 0;
	double kritischerwegvorn = 12; // cm
	boolean detectionecke = false;
	double last_winkel = 0;
	double richtung = 0;
	double xGenau = 0;
	double yGenau = 0;
	int minimalabstand = 11;
	boolean seite_ist_was = false;
	boolean hinten_ist_was = false;
	boolean vorne_ist_was = false;
	boolean detectionactive = false;
	int abstandtriang = 4;
	boolean rechtskurve = false;
	boolean linkskurve = false;
	double alterwinkel = 0;
	double abstand = 0;
	boolean speichern_bereit=false;

	// new
	static Line path = new Line(0, 0, 0, 0);
	double angle3 = 0;
	double angle2 = 0;
	double angle1 = 0;
	int i1 = 0;
	int i2 = 0;

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
		
		//fill paring slot list with dummy slots
		for(int i=0; i< 25; i++)
		{
			list_ParkingSlot[i]=dummy;
		}

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
		this.calculateLocation(); // FEHLER!!!
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
		/**
		 * AngleDifferenceMeasurement rightEncoder =
		 * perception.getControlRightEncoder().getEncoderMeasurement(); //mit
		 * ludwig absprechen an welcher stelle zu platzieren
		 * AngleDifferenceMeasurement leftEncoder =
		 * perception.getControlLeftEncoder().getEncoderMeasurement(); //mit
		 * ludwig absprechen an welcher stelle zu platzieren
		 */

	}

	/**
	 * detektiert die Umwelt des Roboters mittels Triangulationssensoren
	 */
	private void ist_irgendwo_was() {
		if ((this.frontSensorDistance < 13) && (this.frontSensorDistance != 0)) {
			vorne_ist_was = true;
		} else {
			vorne_ist_was = false;
		}
		if (this.backSensorDistance < 8 && this.backSensorDistance != 0) {
			hinten_ist_was = true;
		} else {
			hinten_ist_was = false;
		}
		if (this.frontSideSensorDistance < 13
				&& this.backSideSensorDistance < 13
				&& (this.frontSideSensorDistance != 0)
				&& (this.backSideSensorDistance != 0)) {
			seite_ist_was = true;
		} else {
			seite_ist_was = false;
		}
	}

	//
	//
	// /**
	// * calculates the robot pose from the measurements
	// */
	// /**
	// * calculates the robot pose from the measurements
	// */
	// double phi;
	//
	//
	// private void calculateLocation() {
	//
	//
	// //new
	// if(i1==0){i1 = 5; phi = 0;}
	// //Ausgabe der verwendeten Größen
	// //double xResult = (double)this.angleMeasurementRight.getDeltaT();
	// // double yResult = this.angleMeasurementRight.getAngleSum();
	// // double yResult = (double)this.angleMeasurementLeft.getDeltaT();
	// //anlgeResult wird als Bogenmaß interpretiert und und in ° ausgegeben!
	// // double angleResult = (double)this.angleMeasurementRight.getDeltaT();
	//
	// double leftAngleSpeed = this.angleMeasurementLeft.getAngleSum() /
	// ((double)this.angleMeasurementLeft.getDeltaT()/100000);
	// //degree/seconds
	// double rightAngleSpeed = this.angleMeasurementRight.getAngleSum() /
	// ((double)this.angleMeasurementRight.getDeltaT()/100000);
	// //degree/seconds
	//
	// double vLeft = (leftAngleSpeed * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ;
	// //velocity of left wheel in cm/s
	// double vRight = (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ;
	// //velocity of right wheel in cm/s
	// double w = (vRight - vLeft) / WHEEL_DISTANCE;
	// //angular velocity of robot in rad/s
	//
	// double vMid = (vRight + vLeft) / 2;
	// //velocity of the center of the axis
	// double deltaT = ((double)this.angleMeasurementRight.getDeltaT())/100000;
	// //time-interval
	//
	// //Annahme: vLeft, VRight und deltaT sind während des kurzen
	// Zeitintervalls
	// //konstant. vgl. Mobile Roboter S. 111f.
	//
	// double xResult = this.pose.getX() + vMid * deltaT *
	// Math.cos(this.pose.getHeading() + 0.5 * w * deltaT);
	// double yResult = this.pose.getY() + vMid * deltaT *
	// Math.sin(this.pose.getHeading() + 0.5 * w * deltaT);
	// double angleResult = this.pose.getHeading() + w * deltaT;
	//
	//
	//
	// //Correct Pose through map array.
	// //Be sure, you have the right map!
	// //You have to add in the main method (GuidanceAT):
	// "navigation.setMap(map);"
	// //+90° angle
	// if ((angleResult - phi) < Math.PI &&
	// (angleResult - phi)*180/Math.PI >= 70) {
	// //Referenzwinkel vergrößern
	// phi += Math.PI/2;
	// i2 ++;
	// try{
	// if( i2 > map.length) i2 =0; //FEHLER!!!
	//
	// }catch(Exception e){
	// Sound.beep();
	// Sound.twoBeeps();
	// }
	//
	//
	// if (i2 % 2 == 0) { //position in m.
	// yResult = map[i2].getP1().getY()/100;
	// } else {xResult = map[i2].getP1().getX()/100;}
	// }
	//
	// //-90° angle
	// if ((phi - angleResult) < Math.PI &&
	// (phi - angleResult)*180/Math.PI >= 70 ) {
	// //Referenzwinkel verkleinern
	// phi -= Math.PI/2;
	// i2 ++;
	// if( i2 > map.length) i2 =0;
	// if (i2 % 2 == 0) { //position in m.
	// yResult = map[i2].getP1().getY()/100;
	// } else {xResult = map[i2].getP1().getX()/100;}
	// }
	//
	// //use only angles between 0 and 360°.
	// if(phi >= 2* Math.PI) phi -= 2* Math.PI;
	// if(phi < 0) phi += 2* Math.PI;
	// if(angleResult > 2* Math.PI) angleResult -= 2* Math.PI;
	// if(angleResult < 0) angleResult += 2* Math.PI;
	//
	//
	//
	// //set new pose
	// this.pose.setLocation((float)xResult, (float)yResult);
	// this.pose.setHeading((float)angleResult);
	//
	// }
	//

	// old
	// ----------------------------------------------------------------------------------------------------------

	double phi;

	private void calculateLocation() {

		vorne_ist_was = false;
		seite_ist_was = false;
		hinten_ist_was = false;
		//

		// Umgebung des Roboters detektieren mittels Triangulationssensoren
		ist_irgendwo_was();

		double leftAngleSpeed = this.angleMeasurementLeft.getAngleSum()
				/ ((double) this.angleMeasurementLeft.getDeltaT() / 100000);
		// degree/seconds
		double rightAngleSpeed = this.angleMeasurementRight.getAngleSum()
				/ ((double) this.angleMeasurementRight.getDeltaT() / 100000);
		// degree/seconds

		double vLeft = (leftAngleSpeed * Math.PI * LEFT_WHEEL_RADIUS) / 180;
		// velocity of left wheel in cm/s
		double vRight = (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180;
		// velocity of right wheel in cm/s
		double w = (vRight - vLeft) / WHEEL_DISTANCE;
		// angular velocity of robot in rad/s

		double vMid = (vRight + vLeft) / 2;
		// velocity of the center of the axis
		double deltaT = ((double) this.angleMeasurementRight.getDeltaT()) / 100000;
		// time-interval
		//
		// //Annahme: vLeft, VRight und deltaT sind während des kurzen
		// Zeitintervalls
		// //konstant. vgl. Mobile Roboter S. 111f.
		//
		double xResult = this.pose.getX() + vMid * deltaT
				* Math.cos(this.pose.getHeading() + 0.5 * w * deltaT);
		double yResult = this.pose.getY() + vMid * deltaT
				* Math.sin(this.pose.getHeading() + 0.5 * w * deltaT);
		double angleResult = this.pose.getHeading() + w * deltaT;

		//
		//
		//
		//
		//
		//
		//
		// // if ((seite_ist_was == true)
		// // && (this.frontSideSensorDistance == this.backSideSensorDistance))
		// {
		// // angleResult = phi_kontroll;
		// // } else {
		// // angleResult = this.pose.getHeading() + w * deltaT;
		// // }
		//
		//
		// Anhand des Winkels detektieren ob Kurve vorliegt
		if (this.pose.getHeading() - alterwinkel > +70) {
			detectionecke = true;
			rechtskurve = true;
			linkskurve = false;
		}
//		if ((this.pose.getHeading() - alterwinkel > -70)
//				&& (this.pose.getHeading() != alterwinkel)) {
//			detectionecke = true;
//			rechtskurve = false;
//			linkskurve = true;
//		}
		//
		// }
		// Detektion auf der Linie anhand Abstand zur akt Linie
//		if (detectionecke == false) {
//			Point koordinate = new Point(0, 0);
//			koordinate.setLocation(xResult, yResult);
//
//			for (int i = 0; i < 8; i++) {
//				abstand = Math.abs(map[i].ptLineDist(koordinate));
//				if (abstand < 5) {
//					akt_linie = i;
//				} else {
//					// Nothing to do here!
//
//				}
//			}
//		}

		// Herausfinden einer Ecke anhand der Winkeländerung
		// Wissen an welcher Ecke man sich befindet, anhand der Linie
		if (detectionecke == true) {

			switch (akt_linie) {
			case 0:
				xResult = 180;
				yResult = 0;
				akt_linie++;
				phi_kontroll = 90;
				break;
			case 1:

				xResult = 180;
				yResult = 60;
				akt_linie++;
				phi_kontroll = 180;

				break;
			case 2:

				xResult = 150;
				yResult = 60;
				akt_linie++;
				phi_kontroll = 270;

				break;
			case 3:

				xResult = 150;
				yResult = 30;
				akt_linie++;
				phi_kontroll = 180;

				break;
			case 4:

				xResult = 30;
				yResult = 30;
				akt_linie++;
				phi_kontroll = 90;

				break;
			case 5:

				xResult = 30;
				yResult = 60;
				akt_linie++;
				phi_kontroll = 180;

				break;
			case 6:

				xResult = 0;
				yResult = 60;
				akt_linie++;
				phi_kontroll = 270;

				break;
			case 7:

				xResult = 0;
				yResult = 0;
				akt_linie=0;
				phi_kontroll = 0;

				break;

			default:// no action here

				break;
			}
		}
		//
		// Wenn an Linie 7 angekommen
		
		// if(angleResult > 2* Math.PI) angleResult -= 2* Math.PI;
		// if(angleResult < 0) angleResult += 2* Math.PI;
		
		
		
		alterwinkel = this.pose.getHeading();
		this.pose.setLocation((float) xResult, (float) yResult);
		this.pose.setHeading((float) angleResult);
	}

	/**
	 * detects parking slots and manage them by initializing new slots,
	 * re-characterizing old slots or merge old and detected slots. tatsächliche
	 * Werte der Lücke gespeichert -> nicht nur Pose des Roboters
	 * 
	 * @throws Exception
	 */

	private void detectParkingSlot() throws Exception {
		// Initialisierung
		// double deltaT = ((double) this.angleMeasurementLeft.getDeltaT()) /
		// 1000;

		Point anfang = new Point(0, 0);
		Point ende = new Point(0, 0);
		boolean bekannt = false; // ist die Lücke bekannt?
		int id_bekannte_luecke = 0; // Nummer der Bek. Lücke
		ParkingSlotStatus slotStatus = ParkingSlotStatus.BAD; // SlotStatus am
																// Anfang setzen
																// auf BAD
		double groesse = 0; // Größe der Lücke in cm
		// später 2-fache Messung und Mittelung implementieren

		// double x_abst_backsidesens = 5; // Abstand der Sensoren vom Punkt der
		// // POSE (X,Y)
		// double y_abst_backsidesens = 3; // cm
		// double x_abst_frontsidesens = 5; // cm
		// double y_abst_frontsidesens = 3; // cm
		// Korrektur der Lücke auf wahren Wert um Drehung durch Phi in
		// Abhängigkeit der Linie
		// phi_kontroll in Abhängigkeit der aktuellen Linie
		// double korrektur1 = Math.sin(phi_kontroll) * y_abst_backsidesens
		// - Math.cos(phi_kontroll) * x_abst_backsidesens;
		// double korrektur2 = Math.sin(phi_kontroll) * x_abst_backsidesens
		// - Math.cos(phi_kontroll) * y_abst_backsidesens;
		double distancefront = 0;
		double distanceback = 0;
		double groesse_neu = 0;

		// double
		// korrektur3=Math.sin(this.pose.getHeading())*this.backSideSensorDistance
		// - Math.cos(this.pose.getHeading())*this.backSideSensorDistance;
		// double
		// korrektur4=Math.cos(this.pose.getHeading())*this.backSideSensorDistance
		// - Math.sin(this.pose.getHeading())*this.backSideSensorDistance;

		if (this.frontSideSensorDistance > abstand_sens_band
				&& (detectionactive == false) && this.frontSideSensorDistance!=0 && this.backSideSensorDistance!=0) {

			// pruefe, ob Parkluecke schon vorhanden
			
			// deltax und deltay gegebenenfalls nochmal anpassen
			if(id_aktuell!=0){
			for (int i = 0; i < id_aktuell; i++) {
				double deltax = this.pose.getX() + Math.sin(phi_kontroll)
						* (this.backSideSensorDistance)
						- list_ParkingSlot[i].getFrontBoundaryPosition().getX(); // IN
																					// CM
				double deltay = this.pose.getY()- Math.cos(phi_kontroll)
						* (this.backSideSensorDistance)
						
						- list_ParkingSlot[i].getFrontBoundaryPosition().getY(); // IN
																					// CM
				if ((deltax < 5) && (deltay < 5)) {
					bekannt = true;
					id_bekannte_luecke = i;
					
					// Parklücken später direkt neu vermessen an dieser
					// Stelle und mit alten Daten abgleichen
				}
			}
				} if(bekannt==false) {
					anfang.setLocation(
							this.pose.getX()
									// + korrektur1
									+ Math.sin(phi_kontroll)
									* (this.backSideSensorDistance)
									,
							this.pose.getY()
									// + korrektur2
									- Math.cos(phi_kontroll)
									* (this.backSideSensorDistance)
									);
					detectionactive = true;

					// Lücke vermessen und Speichern (in cm)
				}
			
			
			
			if (this.backSideSensorDistance < abstand_sens_band
					&& detectionactive == true) {
				ende.setLocation(
						this.pose.getX()
								// + korrektur1
								+ Math.sin(phi)
								* (this.frontSideSensorDistance + this.backSideSensorDistance)
								/ 2,
						this.pose.getY()
								// + korrektur2
								- Math.cos(phi)
								* (this.frontSideSensorDistance + this.backSideSensorDistance)
								/ 2);
				detectionactive = false;
				speichern_bereit=true;
			}
				

				// Vergleich mit bekannter Lücke für evtl. Neuvermessung
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
if(speichern_bereit==true){
				// Bewertung
				groesse = Math.abs(anfang.distance(ende));
				// TODO Werte aufnehmen
				if (groesse < 28) {
					// Status ZU KLEIN BAD
					slotStatus = ParkingSlotStatus.BAD;
				} else if ((groesse < 30) && (groesse > 28.1)) {
					// Status auf ungeau -> neu zu vermessen setzen ->
					// kritischer Wert
					slotStatus = ParkingSlotStatus.RESCAN;
				} else {
					// Status auf GUT setzen
					slotStatus = ParkingSlotStatus.GOOD;

				}
				
				if (id_aktuell < 25) {
					Sound.beepSequence();
					list_ParkingSlot[id_aktuell] = new ParkingSlot(id_aktuell,
							anfang, ende, slotStatus);
					LCD.drawString("new Slot " // Ausgabe auf LCD
							+ id_aktuell, 0, 5);
					id_aktuell++;
				} else {
					throw new Exception("No space");
				}
	bekannt=false;		

		} else {
			// NICHTS machen wenn keine Lücke detektiert
		} }

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
