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

	ParkingSlot[] list_ParkingSlot = new ParkingSlot[25]; //create array to save ParkingSlots and send them to HMI

	int abstand_sens_band = 20; // distance from sensor to wall
	int phi_kontroll = 0; // a (2n)*pi/2 angle that is the angle of a line
	int akt_linie = 0; // number of actual line
	int last_linie = 0;

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
	double abstand = 0;
	
	Point anfang = new Point(0, 0);
	Point ende = new Point(0, 0);
	int id_bekannte_luecke = 0; // Nummer der Bek. L�cke##
	
	
	boolean speichern_bereit=false;
	boolean bekannt = false; // gives information, if a slot is known or not
	

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
//		for (int i = 0; i < 25; i++) {		//create some dummy slots for HMI to avoid NullPointException
//			list_ParkingSlot[i] = dummy;
//		}

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

		
		//values given by PERCEPTION in mm -> here: switch them to cm
		this.frontSensorDistance = perception.getFrontSensorDistance()/10;
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance()/10;
		this.backSensorDistance = perception.getBackSensorDistance()/10;
		this.backSideSensorDistance = perception.getBackSideSensorDistance()/10;

	}

	/**
	 * detects the robots environnement with the distancesensors
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

	/**
	 * detects the robots pose from measurements
	 */
	private void calculateLocation() {

		vorne_ist_was = false;
		seite_ist_was = false;
		hinten_ist_was = false; //initialise environnement
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
		// //Annahme: vLeft, VRight und deltaT sind w�hrend des kurzen
		// Zeitintervalls
		// //konstant. vgl. Mobile Roboter S. 111f.
		//
		double xResult = this.pose.getX() + vMid * deltaT
				* Math.cos(this.pose.getHeading() + 0.5 * w * deltaT);
		double yResult = this.pose.getY() + vMid * deltaT
				* Math.sin(this.pose.getHeading() + 0.5 * w * deltaT);
		double angleResult = this.pose.getHeading() + w * deltaT;

		// // if ((seite_ist_was == true)
		// // && (this.frontSideSensorDistance == this.backSideSensorDistance))
		// {
		// // angleResult = phi_kontroll;
		// // } else {
		// // angleResult = this.pose.getHeading() + w * deltaT;
		// // }
		//
		//
		// with the actual change of Heading a corner can be detected
		if (this.pose.getHeading() - alterwinkel > +70) {
			detectionecke = true;
			rechtskurve = true;
			linkskurve = false;
		}
		// if ((this.pose.getHeading() - alterwinkel > -70)
		// && (this.pose.getHeading() != alterwinkel)) {
		// detectionecke = true;
		// rechtskurve = false;
		// linkskurve = true;
		// }
		//
		// }
		// Detektion auf der Linie anhand Abstand zur akt Linie
		// if (detectionecke == false) {
		// Point koordinate = new Point(0, 0);
		// koordinate.setLocation(xResult, yResult);
		//
		// for (int i = 0; i < 8; i++) {
		// abstand = Math.abs(map[i].ptLineDist(koordinate));
		// if (abstand < 5) {
		// akt_linie = i;
		// } else {
		// // Nothing to do here!
		//
		// }
		// }
		// }

		// know on which line the roboter is -> when a corner was detected -> line switch and known points are set as actual pose
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
				akt_linie = 0;
				phi_kontroll = 0;

				break;

			default:// no action here

				break;
			}
		}
		//
		// When he arrived at Line No 7 -> start at 0!

		if (angleResult > 2 * Math.PI)
			angleResult -= 2 * Math.PI;
		if (angleResult < 0)
			angleResult += 2 * Math.PI;

		alterwinkel = this.pose.getHeading();
		this.pose.setLocation((float) xResult, (float) yResult);
		this.pose.setHeading((float) angleResult);
	}

	/**
	 * detects parking slots and manage them by initializing new slots,
	 * re-characterizing old slots or merge old and detected slots. tats�chliche
	 * Werte der L�cke gespeichert -> nicht nur Pose des Roboters
	 * 
	 * @throws Exception
	 */
	//FIXME 
	// Exception is possible -> when array is full and no more ParkingSlots can be detected!
	private void detectParkingSlot() throws Exception {
		// initialise
		// double deltaT = ((double) this.angleMeasurementLeft.getDeltaT()) /
		// 1000;

		ParkingSlotStatus slotStatus = ParkingSlotStatus.BAD; // first, set SlotStatus to BAD
		double groesse = 0; // Size of the Slot in cm
		// sp�ter 2-fache Messung und Mittelung implementieren

		// double x_abst_backsidesens = 5; // Abstand der Sensoren vom Punkt der
		// // POSE (X,Y)
		// double y_abst_backsidesens = 3; // cm
		// double x_abst_frontsidesens = 5; // cm
		// double y_abst_frontsidesens = 3; // cm
		// Korrektur der L�cke auf wahren Wert um Drehung durch Phi in
		// Abh�ngigkeit der Linie
		// phi_kontroll in Abh�ngigkeit der aktuellen Linie
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

		// detects if a ParkingSlot is available
		if ((this.frontSideSensorDistance > abstand_sens_band)
				&& (this.frontSideSensorDistance != 0)
				&& (this.backSideSensorDistance != 0)&&(detectionactive==false)) {
			
			// check if Slot is still existing
			// deltax und deltay has to be checked by experiments
			if (id_aktuell != 0) {
				for (int i = 0; i < id_aktuell; i++) {
					double deltax = this.pose.getX()
							+ Math.sin(phi_kontroll)
							* (this.backSideSensorDistance)
							- list_ParkingSlot[i].getFrontBoundaryPosition()
									.getX(); // IN
												// CM
					double deltay = this.pose.getY()
							- Math.cos(phi_kontroll)
							* (this.backSideSensorDistance)

							- list_ParkingSlot[i].getFrontBoundaryPosition()
									.getY(); // IN
												// CM
					if ((deltax < 5) && (deltay < 5)) {
						bekannt = true;
						id_bekannte_luecke = i;
//later: compare Slots and check if new measurement is necessary
					}
				}
			}
		//save frontBoundary
			if (bekannt == false) {
				anfang.setLocation(this.pose.getX()
						// + korrektur1
						+ Math.sin(phi_kontroll)
						* 15, this.pose.getY()
						// + korrektur2
						- Math.cos(phi_kontroll)
						* 15);
				detectionactive = true;
				

				// measure the Slot and save
			}
		}
		
		//check BackBoundary!
			//here the measurements are improved by adding them and dividing by 2
			
			if ( (this.frontSideSensorDistance <= abstand_sens_band)
					&& detectionactive == true) { 
				
				
				
				ende.setLocation(
						this.pose.getX()
								// + korrektur1
								+ Math.sin(phi_kontroll)
								* 15
								,
						this.pose.getY()
								// + korrektur2
								- Math.cos(phi_kontroll)
								* 15
								);
				//this.frontSideSensorDistance an stelle von 15!! damit abh�ngig von Sensor L�cke gezeihnet wird
				speichern_bereit = true;
			}

			// compare with known slot -> maybe a new measurement is necessary
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
			if (speichern_bereit == true) {
				// judge the slot
				groesse = Math.abs(anfang.distance(ende));
				// TODO Werte aufnehmen
				if (groesse < 28) {
					// too small for robot -> switch state to BAD
					slotStatus = ParkingSlotStatus.BAD;
				} else if ((groesse < 30) && (groesse > 28.1)) {
					// switch state to RESCAN because it is in a critical intervall
					slotStatus = ParkingSlotStatus.RESCAN;
				} else {
					// switch state to GOOD
					slotStatus = ParkingSlotStatus.GOOD;

				}

				if (id_aktuell < 25) {
					
					
					list_ParkingSlot[id_aktuell] = new ParkingSlot(id_aktuell,
							anfang, ende, slotStatus);
				//	LCD.drawString("new Slot " // write new SLOT on LCD
					//		, 0, 5);
					id_aktuell++;
					speichern_bereit=false;
					detectionactive=false;
					bekannt=false;
				} else {
					throw new Exception("No space"); // Exception thrown if array is full
				}
			
			} else {
				// NICHTS machen wenn keine L�cke detektiert
			}
			return;
		}



	/**
	 * 
	 * @param i
	 *            ID of the certain SLOT
	 * @return ParkingSlot from the ARRAY with ID for HMI
	 */
	public ParkingSlot getSlotById(int i) {
		return list_ParkingSlot[i];
	}
//	return;

}
