package parkingRobot.hsamr8;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import lejos.robotics.navigation.Pose;
import lejos.geom.Line;
import lejos.nxt.LCD;
import parkingRobot.hsamr8.ControlRST;
import parkingRobot.hsamr8.HmiPLT;
import parkingRobot.hsamr8.NavigationAT;
import parkingRobot.hsamr8.PerceptionPMP;

/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
 * parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
 * structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
 * state machines (FSM). It may be advantageous to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be realized in one main module class.
 * Every class (except guidance) has additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module class thread is only handled in a
 * synchronized context to avoid inconsistent or corrupt data!
 */
public class GuidanceAT {
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		DRIVING,
		/**
		 * indicates that robot is paused
		 */
		INACTIVE,
		/**
		 * indicates that robot is performing a parking maneuver with determined parking slot
		 */
		PARK_THIS,
		/**
		 * indicates that robot is perfroming a parking maneuver as soon as possible
		 */
		PARK_NOW,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT		
	}
	
	
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;

	
	/**
	 * Substates for the parallel state machine. 
	 * Not supposed to be sent to the tablet, needed to realize submissions
	 *
	 */
	private enum CurrentSubStatus {
		/**
		 * State in which the robot follows the black line.
		 * Collision detection and parking slot detection are active.
		 * Allowed during DRIVING.
		 */
		FOLLOW_LINE,
		/**
		 * The robot has detected an object less than 10cm ahead and stops.
		 * Allowed during DRIVING.
		 */
		WAIT,
		/**
		 * The robot has detected an object less than 7cm ahead and evades.
		 * Allowed during DRIVING.
		 */
		DRIVE_BACKWARDS,
		/**
		 * The robot is currently off line and finds its way back.
		 * Allowed during DRIVING.
		 */
		FIND_BLACK_LINE,
		/**
		 * State in which the robot awaits new orders while standing still.
		 * The location is close enough to the black line to activate line following.
		 * Allowed during INACTIVE.
		 */
		WAITING_ON_LINE,
		/**
		 * State in which the robot awaits new orders while standing still.
		 * The location far from the black line, line following would cause misbehaviour.
		 * Allowed during INACTIVE.
		 */
		WAITING_OFF_LINE,
		/**
		 * The robot follows the black line and compares its location to 
		 * Coordinates of known parking slots.
		 * Collision detection and parking slot detection are active.
		 * Allowed during PARK_NOW.
		 */
		SEARCH,		
		/**
	 	 * The robot moves towards a position, from where a parking 
	 	 * Maneuver is supposed to start. During PARK_NOW, the robot decides
	 	 * Whether its better to move backwards rather than forward.
	 	 * Collision detection and parking slot detection are active.
	 	 * Allowed during PARK_NOW and PARK_THIS.
		 */
		MOVE_TO_PARKING_POSITION,
		/**
		 * The pathgenerator is activated and the robots drives into a parking slot.
		 * Collision detection and parking slot detection are active.
		 * Allowed during allowed during PARK_NOW and PARK_THIS.
		 */
		PARK	
	}
	
	/**
	 * state in which the parallel state machine is running at the moment
	 */
	protected static CurrentSubStatus currentSubStatus 	= CurrentSubStatus.WAITING_ON_LINE;
	/**
	 * state in which the parallel state machine was running before entering the actual state
	 */
	protected static CurrentSubStatus lastSubStatus		= CurrentSubStatus.WAITING_ON_LINE;
	
	
	/**
	 * Converts currentSubStatus to String. Needed to show current Status on display.
	 * Possibly only needed for testing
	 * @return
	 */
	private static String CurrentSubStatusAsString(){
		String currentSubStatusString = null;
		switch (currentSubStatus){
			case FOLLOW_LINE:
				currentSubStatusString = "FOLLOW_LINE";
				break;
			case WAIT:
				currentSubStatusString = "WAIT";
				break;
			case DRIVE_BACKWARDS:
				currentSubStatusString = "DRIVE_BACKWARDS";
				break;
			case FIND_BLACK_LINE:
				currentSubStatusString = "FIND_BLACK_LINE";
				break;
			case WAITING_ON_LINE:
				currentSubStatusString = "WAITING_ON_LINE";
				break;
			case WAITING_OFF_LINE:
				currentSubStatusString = "WAITING_OFF_LINE";
				break;
			case SEARCH:
				currentSubStatusString = "SEARCH";
				break;
			case MOVE_TO_PARKING_POSITION:
				currentSubStatusString = "MOVE_TO_PARKING_POSITION";
				break;
			case PARK:
				currentSubStatusString = "PARK";
				break;
			default:
				currentSubStatusString = "UNKNOWN";
				break;
 		}
		return currentSubStatusString;
	}
	
	/**
	 * one line of the map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * This documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(  0,  0, 180,  0);
	static Line line1 = new Line(180,  0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30,  30, 30);
	static Line line5 = new Line( 30, 30,  30, 60);
	static Line line6 = new Line( 30, 60,   0, 60);
	static Line line7 = new Line(  0, 60,   0,  0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};

		
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {			
		
		//ORIGINAL STARTZUSTAENDE
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus    = CurrentStatus.EXIT;
				
//		//TestStates PARK NOW + SEARCH
//		currentStatus = CurrentStatus.PARK_NOW;
//		lastStatus    = CurrentStatus.PARK_NOW;
//        currentSubStatus = CurrentSubStatus.SEARCH;
//        lastSubStatus = CurrentSubStatus.FOLLOW_LINE;


//		//TestStates PARK NOW + MOVE_TO_PARKING_POSITION
//		currentStatus = CurrentStatus.PARK_NOW;
//		lastStatus    = CurrentStatus.PARK_NOW;
//        currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
//        lastSubStatus = CurrentSubStatus.SEARCH;

//		//TestStates PARK NOW + PARK
//		//also good for testing trajectory
//		currentStatus = CurrentStatus.PARK_NOW;
//		lastStatus    = CurrentStatus.PARK_NOW;
//        currentSubStatus = CurrentSubStatus.PARK;
//        lastSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;

//		//TestStates DRIVE BACKWARDS
//		//also used to test control functions
//		currentStatus = CurrentStatus.DRIVING;
//		lastStatus    = CurrentStatus.DRIVING;
//		currentSubStatus = CurrentSubStatus.DRIVE_BACKWARDS;
//		lastSubStatus = CurrentSubStatus.WAIT;	

		
		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);
		
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor);
//		perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT(perception);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control);
		navigation.setMap(map);	

		
		while(true) {
			showData(navigation, perception, control);
			
        	switch ( currentStatus )
        	{          	
        		case DRIVING:
					
					main_state_transition_check(hmi);
										
					//parallel state machine DRIVING
					switch ( currentSubStatus ) {
					
						case FOLLOW_LINE:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.FOLLOW_LINE ){
								navigation.setDetectionState(true);
								navigation.setParkingActive(false);
								control.setCtrlMode(ControlMode.LINE_CTRL);
							}
							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
								currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
							}else if (currentStatus == CurrentStatus.PARK_NOW){
								currentSubStatus = CurrentSubStatus.SEARCH;
							}
							break;	
							
						case WAIT:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.WAIT ){
								navigation.setDetectionState(false);
								navigation.setParkingActive(false);
								control.setCtrlMode(ControlMode.INACTIVE);
							}
							//State transition check; orders are overwritten!
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.INACTIVE)||(currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentStatus = CurrentStatus.DRIVING;
							}
							//Collision check	
							if ((perception.getFrontSensorDistance() < 70)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.DRIVE_BACKWARDS;
							}else if ((perception.getFrontSensorDistance() > 150)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}
							break;	
							
						case DRIVE_BACKWARDS:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.DRIVE_BACKWARDS ){
								//navigation states do not change coming from WAIT
								control.setVelocity(-10);
								control.setAngularVelocity(0);	
								control.setCtrlMode(ControlMode.VW_CTRL);
							}
							
							//State transition check; orders are overwritten
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.INACTIVE)||(currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentStatus = CurrentStatus.DRIVING;
							}								
							//Collision check
							if ((perception.getFrontSensorDistance() > 100)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.WAIT;
							}
							break;	
							
						case FIND_BLACK_LINE:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.FIND_BLACK_LINE ){
								setDestination(control);
								navigation.setDetectionState(false);
								navigation.setParkingActive(true);
								control.drive_backwards(false);
								control.setCtrlMode(ControlMode.PARK_CTRL);
								
//								//orders needed to test SetPose()
//								navigation.setParkingActive(true);
//								control.setDestination(0,0,1);
//								control.setCtrlMode(ControlMode.SETPOSE);
//								navigation.setDetectionState(false);
							}					

							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}else if ((currentStatus == CurrentStatus.PARK_NOW)||(currentStatus == CurrentStatus.PARK_THIS)){
								//the states are allowed, but the robot has to find the line first
								currentStatus = CurrentStatus.DRIVING;
							}
							
							//check, if black line is reached
							if(control.destination_reached()){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}
							break;	
							
						default:
							//this should never happen
							break;
					}
					//end of "case DRIVING"
					break;
					
				case INACTIVE:
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						navigation.setDetectionState(false);
						control.setCtrlMode(ControlMode.INACTIVE);
					}

					main_state_transition_check(hmi);
					
					//parallel state machine INACTIVE				
					switch ( currentSubStatus ) {
						case WAITING_ON_LINE:
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
								currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
							}else if (currentStatus == CurrentStatus.PARK_NOW){
								currentSubStatus = CurrentSubStatus.SEARCH;
							}
							break;	
							
						case WAITING_OFF_LINE:
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
							}else if ((currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								//since this state is also activated after a successful parking maneuver,
								//the states where the machine came from are overwritten to suppress
								//instant leaving of parking slot
								currentStatus = CurrentStatus.INACTIVE;			
							}
							break;
							
						default:
							//this should never happen
							break;
						}
					//end of "case INACTIVE"
					break;
					
				case PARK_THIS:
					
					main_state_transition_check(hmi);
					
					//parallel state machine PARK_THIS					
					switch ( currentSubStatus ) {	
						case MOVE_TO_PARKING_POSITION:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION ){																		
								//in PARK_THIS, hmi sends the ID of the destined parking slot
								destination = navigation.getParkingSlots()[hmi.getSelectedParkingSlot()];
								navigation.setDetectionState(true);
								navigation.setParkingActive(false);
								setDestination(control);
								control.setCtrlMode(ControlMode.LINE_CTRL);
							}

							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_NOW){
								currentSubStatus = CurrentSubStatus.SEARCH;
							}
							
							//check, if parking position is reached
							if(x_reached(navigation)&&(y_reached(navigation))){
								currentSubStatus = CurrentSubStatus.PARK;
							}
							break;	
							
						case PARK:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.PARK ){
								setDestination(control);
								navigation.setDetectionState(false);
								navigation.setParkingActive(true);
								control.drive_backwards(true);
								control.setCtrlMode(ControlMode.PARK_CTRL);
							}
					
							//check, if parking slot is reached
							if(control.destination_reached()){
								currentStatus = CurrentStatus.INACTIVE;
								Sound.beepSequenceUp();
							}				
				
							//State transition check
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.DRIVING)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}
							
							break;
							
						default:
							//this should never happen
							break;
						}
					//end of "case PARK_THIS"
					break;
					
				case PARK_NOW:
					
					main_state_transition_check(hmi);
					
					//parallel state machine PARK_NOW				
					switch ( currentSubStatus ) {
						case SEARCH:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.SEARCH ){
								navigation.setDetectionState(true);
								navigation.setParkingActive(false);
								control.setCtrlMode(ControlMode.LINE_CTRL);
							}	
																					
							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else 
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
							currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
							}
							
							//compare known parking slots with current position
							//move to parking position if parking slot is close							
							{
								ParkingSlot[] parkingSlots = navigation.getParkingSlots();
								Pose currentPose = navigation.getPose();
								for (int i=0; i<parkingSlots.length; i++){
				
									//test if parking slot exists
									if (parkingSlots[i]!=null){
										
										//test if parking slot has status 'GOOD'
										if (parkingSlots[i].getStatus().ordinal()==0){      
											
											//calculate the parking position of this slot
											lejos.geom.Point current_checking = parkingSlots[i].getBackBoundaryPosition();
											float phi = 0;
											
											if(current_checking.getY()<0){
												phi = 0;
											}else if(current_checking.getX()>1.8){
												phi = (float)Math.PI*1/2;
											}else if(current_checking.getY()>0.5){
												phi = (float)Math.PI;
											}else if(current_checking.getX()<0){
												phi = (float)Math.PI*3/4;
											}
											double distance_slot_to_line = 0.3;
											double current_checking_X = current_checking.getX() - distance_slot_to_line*Math.sin(phi);
											double current_checking_Y = current_checking.getY() + distance_slot_to_line*Math.cos(phi);
											
											double deltaX = Math.abs(current_checking_X-currentPose.getX());
											double deltaY = Math.abs(current_checking_Y-currentPose.getY());
											
											//test if robot is close to parking slot
											double tollerance = 0.15;
											if((deltaX<tollerance)&&(deltaY<tollerance)){
												destination = navigation.getParkingSlots()[i];
												currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
												Sound.buzz();
												break;
											}
										}
									}																							
								}
							}		
							break;
							
						case MOVE_TO_PARKING_POSITION:
							//Into action							
							if ( lastSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION ){																		
								setDestination(control);	
								
								control.setCtrlMode(ControlMode.INACTIVE);
								Thread.sleep(200);
								
							
								
								//catch the case, that the detected parking slot is close, but behind the robot
								boolean forward = true;
								switch (navigation.getLine()){
									case 0:
										if (destination_pose.getX()-navigation.getPose().getX()<0)
										{forward = false;}
										break;
									case 1:
										if (destination_pose.getY()-navigation.getPose().getY()<0)
										{forward = false;}
										break;
									case 2:
										//this is the case, if the robot just detected a parking slot on next to line 1
										//but already passed the corner at (x|y)=(180|60)
										if (destination_pose.getX()-navigation.getPose().getX()>0)
										{forward = false;}
										break;
									case 4:
										if (destination_pose.getX()-navigation.getPose().getX()>0)
										{forward = false;}
										break;
									default:
										//this should not happen
										break;
								}
								
								//decision whether to move forward or backwards
								if(forward) {
									navigation.setDetectionState(true);
									navigation.setParkingActive(false);
									control.setCtrlMode(ControlMode.LINE_CTRL);
								}else{
									navigation.setDetectionState(false);
									navigation.setParkingActive(true);
									if (navigation.getLine()==2){
										//this catches the case, that the robot already passed the 
										//second corner and driving backwards wouldn't lead to a good result
										control.setDestination(destination_pose.getHeading(),destination_pose.getX(),destination_pose.getY());
										control.setCtrlMode(ControlMode.SETPOSE);
									} else {
										
										control.setVelocity(-10);
										control.setAngularVelocity(0);
										control.setCtrlMode(ControlMode.VW_CTRL);
									}
								}
							}
				
							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else 
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
								//needed for "into-actions"
								lastSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}
							
							//check, if parking position is reached
							if(x_reached(navigation)){
								if(phi_reached(navigation)){
									double theta_f = destination_pose.getHeading();
									Pose currentPosition = navigation.getPose();
								//catch special case: phi_dest = 0 -> phi_current might be higher than 180∞
								if ((Math.abs(theta_f)<Math.PI*5/180)&&(currentPosition.getHeading()>Math.PI)){
									 theta_f = Math.PI*2;
									 }
								double delta_phi = currentPosition.getHeading()-theta_f;
								if 	(Math.abs(delta_phi)>Math.PI*4/180){
									if (delta_phi<0){
										control.setVelocity(0);
										control.setAngularVelocity(-0.7);
										control.setCtrlMode(ControlMode.VW_CTRL);
									} else if (delta_phi>0){
										control.setVelocity(0);
										control.setAngularVelocity(+0.7);
										control.setCtrlMode(ControlMode.VW_CTRL);
									} 
								} 
								 } else{
									
							
								currentSubStatus = CurrentSubStatus.PARK;} } 
							break;	
							
						case PARK:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.PARK ){
								
//								//set fixed destination, for testing only
//								//needs DUMMY parking slots in Navigation
//								destination = navigation.getParkingSlots()[0];
								
								setDestination(control);
								navigation.setDetectionState(false);
								navigation.setParkingActive(true);
								control.drive_backwards(true);
								control.setCtrlMode(ControlMode.PARK_CTRL);
							}
							
							//check if parking slot is reached
							if(control.destination_reached()){
								currentStatus = CurrentStatus.INACTIVE;
								Sound.beepSequenceUp();
							}				
				
							//State transition check
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.DRIVING)||(currentStatus == CurrentStatus.PARK_THIS)){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}
							break;	
							
						default:
							//this should never happen
							break;
						}
					//end of "case PARK_NOW"
					break;
					
				case EXIT:
					hmi.disconnect();
					System.exit(0);
					break;
					
				default:
					//this should never happen
					break;
        	}
        	
        	Thread.sleep(100);        	
		}
	}
	
	
	/**
	 * State transition check. The current state of the machine is 
	 * changed according to incoming orders from hmi or NXT-Buttons,
	 *  used in every state except EXIT.
	 */
	private static void main_state_transition_check(INxtHmi hmi)throws Exception{
		lastStatus = currentStatus;
		if ( Button.ENTER.isDown() ){
			//if the robot waited before, it should move now
			//if it moved, it's supposed to stop
			switch (currentStatus){
				case INACTIVE:
					currentStatus = CurrentStatus.DRIVING;
					break;
				default:
					currentStatus = CurrentStatus.INACTIVE;
					break;
			}
			while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
		}else if ( Button.ESCAPE.isDown() ){
			currentStatus = CurrentStatus.EXIT;
			while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
		}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
			currentStatus = CurrentStatus.EXIT;
		} else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
			currentStatus = CurrentStatus.DRIVING;
		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
			currentStatus = CurrentStatus.INACTIVE;						
		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS ){
			currentStatus = CurrentStatus.PARK_THIS;
		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW ){
			currentStatus = CurrentStatus.PARK_NOW;
		}
		Thread.sleep(30);
	}
	
	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return currentStatus;
	}
	
	/**
	 * Plots information on the robots display.
	 * Mainly used for testing and finding logical errors.
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 * @param perception reference to the perception class for getting sensor information
	 * @param control reference to the control class for getting information
	 */
	protected static void showData(INavigation navigation, IPerception perception,IControl control)throws Exception{
		LCD.clear();	
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
		LCD.drawString( CurrentSubStatusAsString() , 0, 3);
		LCD.drawString("LINE NO " + (navigation.getLine()), 0, 4);
		LCD.drawString("PHI_SOLL " + (navigation.getPhi()), 0, 5);
		LCD.drawString("PHI_dest: " + (destination_pose.getHeading()/Math.PI*180), 0, 6);
		LCD.drawString("sensor " + perception.getRightLineSensorValue(), 0, 7);
//		if (navigation.getParkingSlots()[3]!=null){
//		LCD.drawString("X Front " + navigation.getParkingSlots()[3].getFrontBoundaryPosition().getX(),0,4);
//		LCD.drawString("Y Front " + navigation.getParkingSlots()[3].getFrontBoundaryPosition().getY(),0,5);
//		LCD.drawString("X Back " + navigation.getParkingSlots()[3].getBackBoundaryPosition().getX(),0,6);
//		LCD.drawString("Y Back " + navigation.getParkingSlots()[3].getBackBoundaryPosition().getY(),0,7);
//		}
//		perception.showSensorData();
		
//    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
//			LCD.drawString("HMI Mode SCOUT", 0, 3);
//		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
//			LCD.drawString("HMI Mode PAUSE", 0, 3);
//		}else{
//			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
//		}
		Thread.sleep(20);
	}
	
	/**
	 *ParkingSlot to be driven to. Not necessarily the same position as destination_pose.
	 */
	private static ParkingSlot destination = null;
	/**
	 *destination to be driven to. Not necessarily the same position as destination (ParkingSlot).
	 */
	private static Pose destination_pose = new Pose();
	
/**
 * 	sets destination_pose dependent on current sub state and destination (ParkingSlot)
 * @param control reference to the control class for setting calculated destination
 * @throws Exception
 */
		private static void setDestination(IControl control)throws Exception{
			//coordinates of parking slot
			double back_X = destination.getBackBoundaryPosition().getX();
			double back_Y = destination.getBackBoundaryPosition().getY();
			double front_X = destination.getFrontBoundaryPosition().getX();
			double front_Y = destination.getFrontBoundaryPosition().getY();
			//midpoint of parking slot
			double middle_X = (back_X+front_X)/2;
			double middle_Y = (back_Y+front_Y)/2;
			float phi = 0;
			
			if(middle_Y<0){
				phi = 0;
			}
			if(middle_X>1.8){
				phi = (float)Math.PI*1/2;
			}
			if(middle_Y>0.5){
				phi = (float)Math.PI;
			}
			//coordinates on black line, which are supposed to be driven to before parking
			double distance_slot_to_line = 0.3;
			double distance_to_go = 0.2;
			double line_X = middle_X + distance_to_go*Math.cos(phi) - distance_slot_to_line*Math.sin(phi);
			double line_Y = middle_Y + distance_slot_to_line*Math.cos(phi) + distance_to_go*Math.sin(phi);
			
			//offset to improve parking behaviour
			double middle_X_co = middle_X + 0.1*Math.cos (phi);
			double middle_Y_co = middle_Y - 0.1*Math.sin (phi);
			
			//avoid parking maneuver while in the middle of the second corner
			if (line_Y>0.55){
				line_Y = line_Y-0.03;
			}
			destination_pose.setHeading(phi);
			
			//control is only told the the destination if a parking maneuver is supposed to start
			//this triggers the calculation of the trajectory
			if((currentSubStatus == CurrentSubStatus.MOVE_TO_PARKING_POSITION)||(currentSubStatus == CurrentSubStatus.FIND_BLACK_LINE)){
				destination_pose.setLocation((float)line_X, (float)line_Y);
			}
			if(currentSubStatus == CurrentSubStatus.PARK){
				destination_pose.setLocation((float)middle_X_co, (float)middle_Y_co);
				control.setDestination(phi, middle_X_co, middle_Y_co);
			} else {
				//make sound if method fails
				Sound.buzz();;
			}
		}
		
		/**
		 * compares current position with destined position and returns true if close to destination
		 * @param navigation reference to the navigation class for getting pose information
		 * @return true if destined coordinates are reached
		 * @throws Exception
		 */
		private static boolean x_reached(INavigation navigation)throws Exception{
			boolean destination_reached = false;
			Pose current_position = navigation.getPose();
			float current_X = current_position.getX();
//			float current_Y = current_position.getY();
//			float current_phi = current_position.getHeading();
			double tollerance = 0.04;
			//compare current position with destined position
			if((Math.abs(current_X-destination_pose.getX())<tollerance)){//&&(Math.abs(current_Y-destination_pose.getY())<tollerance)&&(Math.abs(current_phi-destination_pose.getHeading())<100*tollerance/180*Math.PI)){
				Sound.beep();
				destination_reached = true;
			}
			return destination_reached;
		}	
		/**
		 * compares current position with destined position and returns true if close to destination
		 * @param navigation reference to the navigation class for getting pose information
		 * @return true if destined coordinates are reached
		 * @throws Exception
		 */
		private static boolean y_reached(INavigation navigation)throws Exception{
			boolean destination_reached = false;
			Pose current_position = navigation.getPose();
//			float current_X = current_position.getX();
			float current_Y = current_position.getY();
//			float current_phi = current_position.getHeading();
			double tollerance = 0.04;
			//compare current position with destined position
			if((Math.abs(current_Y-destination_pose.getY())<tollerance)){//if((Math.abs(current_X-destination_pose.getX())<tollerance)&&(Math.abs(current_Y-destination_pose.getY())<tollerance)&&(Math.abs(current_phi-destination_pose.getHeading())<100*tollerance/180*Math.PI)){
				Sound.beep();
				destination_reached = true;
			}
			return destination_reached;
		}	
		/**
		 * compares current position with destined position and returns true if close to destination
		 * @param navigation reference to the navigation class for getting pose information
		 * @return true if destined coordinates are reached
		 * @throws Exception
		 */
		private static boolean phi_reached(INavigation navigation)throws Exception{
			boolean destination_reached = false;
			Pose current_position = navigation.getPose();
			float current_X = current_position.getX();
			float current_Y = current_position.getY();
			float current_phi = current_position.getHeading();
			double tollerance = 0.04;
			//compare current position with destined position
			if((Math.abs(current_phi-destination_pose.getHeading())<100*tollerance/180*Math.PI)){//if((Math.abs(current_X-destination_pose.getX())<tollerance)&&(Math.abs(current_Y-destination_pose.getY())<tollerance)&&(Math.abs(current_phi-destination_pose.getHeading())<100*tollerance/180*Math.PI)){
				Sound.beep();
				destination_reached = true;
			}
			return destination_reached;
		}	
	
		/**
		 * method to detect objects, which are less than 10cm ahead. 
		 * If so, the currentState and currentSubState are overwritten, the robot stops moving
		 * @param perception reference to the perception class for getting sensor information
		 * @return returns true, if an object is detected less than 10cm ahead
		 */
		private static boolean collision_detection(IPerception perception){
			boolean collision = false;
			if ((perception.getFrontSensorDistance() < 100) && (perception.getFrontSensorDistance()!=0)){
				Sound.twoBeeps();
				collision = true;
				currentSubStatus = CurrentSubStatus.WAIT;
				currentStatus = CurrentStatus.DRIVING;
			}
			return collision;
		}
}
