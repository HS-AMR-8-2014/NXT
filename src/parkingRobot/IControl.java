package parkingRobot;

import lejos.robotics.navigation.Pose;

/**
 * interface for the main module 'Control', providing methods for executing the algorithms to
 * control the robot according to different modes which is set by guidance.
 * 
 * @author RST
 */
public interface IControl {
	// Inputs
	
	
	/**
	 * The predefined control modes
	 */
	public enum ControlMode {
		
		/**
		 * follow black line
		 */
		LINE_CTRL,
		
		/**
		 * park
		 */
		PARK_CTRL,
		
		/**
		 * v/w-Control
		 */
		VW_CTRL,
		
		/**
		 * drive to a given destination
		 */
		SETPOSE,
		
		/**
		 * NXT standby
		 */
		INACTIVE
	}
	
		
	/**
	 * set the required speed
	 * 
	 * @param velocity the velocity of the robot to be set
	 */	
	public void setVelocity(double velocity);

	
	/**
	 *  set the required angular velocity
	 *  
	 * @param angularVelocity  the angular velocity to be set
	 */
	public void setAngularVelocity(double angularVelocity);

	/**
	 * set the destination to be driven to
	 * 
	 * @param heading the heading angle of the robot at the destination
	 * @param x the destination position in x axis
	 * @param y the destination position in y axis
	 */
	public void setDestination(double heading, double x, double y);
		
	
	/**
	 * the Robot's current position 
	 * 
	 * @param currentPosition the current position of the robot at each sampling  
	 */	
	public void setPose(Pose currentPosition); 	
	

	
	
	/**
	 * set the current control mode
	 * 
	 * @param ctrl_mode parameter for control mode which is defined by Guidance 
	 */
	public void setCtrlMode(ControlMode ctrl_mode);
	
	
	
	
	/**
	 * get actual CNTRLMode
	 * for test
	 * @return 
	 */
	public ControlMode getCtrlMode();
	
	/**
	 * set start time
	 * @param startTime
	 */
	public void setStartTime(int startTime);
	
	
	/**
	 * execute the selected algorithms for control which was set by guidance
	 */
	public void exec_CTRL_ALGO();

	/**
	 * method for Park_CTRL, sets a variable true, if currentPosition=destination
	 */
	public boolean destination_reached();

	/**
	 * used to set the state of driving backwards
	 */
	public void drive_backwards(boolean isOn);
	/**
	 * test method to get the current velocity of the right wheel
	 */
	public double get_rvelocity();
	/**
	 * test method to get the current velocity of the left wheel
	 */
	public double get_lvelocity();
}

