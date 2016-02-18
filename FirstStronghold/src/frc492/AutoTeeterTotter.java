package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcRobot.AutoStrategy;

public class AutoTeeterTotter implements AutoStrategy 
	{
		private static final String moduleName = "AutoTeeterTotter";
	 	private Robot robot;
	 	private TrcEvent driveEvent;
	 	private TrcEvent armEvent;
	 	private TrcStateMachine sm;
	 	
		public enum AutonomousState
	 	{
	 		STATE_MOVE_ROBOT_FAST,
	 		STATE_MOVE_ARM_DOWN,
	 		STATE_MOVE_ROBOT_SLOWLY,
	 		STOP
	 	}
	 	
	      public AutoTeeterTotter(Robot robot)
	    {
	        this.robot = robot;
	        driveEvent = new TrcEvent (moduleName + ".driveEvent");
	        armEvent = new TrcEvent (moduleName + ".armEvent");
	        sm = new TrcStateMachine (moduleName + ".sm");
	        sm.start(AutonomousState.STATE_MOVE_ROBOT_FAST);
	    }
	      
	      private static final double DISTANCE_TO_TEETERTOTTER = 1; //giving values move to robot info
	      private static final double HIGH_SPEED_POWER = 0.7;
	      private static final double SLOW_SPEED_POWER = 0.3;
	      private static final double ARM_GROUND_POSITION = 0;
	      
	      
	      public void autoPeriodic(double elapsedTime) 
	  	{    	
	  		
	  		if (!sm.isReady())
	  			return;
	  		
	  		AutonomousState state = (AutonomousState)sm.getState();
	  		
	  		switch(state)
	  		{
	  		case STATE_MOVE_ROBOT_FAST:
	  			robot.encoderYPidCtrl.setOutputRange(-HIGH_SPEED_POWER, HIGH_SPEED_POWER);
	  			robot.pidDrive.setTarget(0.0, DISTANCE_TO_TEETERTOTTER * (5/6), 0, false, driveEvent, 1);
	  			sm.waitForEvents(AutonomousState.STATE_MOVE_ARM_DOWN);
	  			break;
	  			
	  		case STATE_MOVE_ARM_DOWN:
	  			robot.arm.setPosition(ARM_GROUND_POSITION, armEvent, 0.0);
	  			sm.waitForEvents(AutonomousState.STATE_MOVE_ROBOT_SLOWLY);
	  			break;
	  			
	  		case STATE_MOVE_ROBOT_SLOWLY:
	  			robot.encoderYPidCtrl.setOutputRange(-SLOW_SPEED_POWER, SLOW_SPEED_POWER);
	  			robot.pidDrive.setTarget(0.0, DISTANCE_TO_TEETERTOTTER * (1/6), 0, true, driveEvent, 2);
	  			sm.waitForEvents(AutonomousState.STOP);
	  			break;
	  		
	  		case STOP:
	  		default:
	  		    robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
	  			sm.stop ();
	  		
	  		}
	  		
	  	}
	  	
	  }//End of class AutoTeeterTotter

	  
	

