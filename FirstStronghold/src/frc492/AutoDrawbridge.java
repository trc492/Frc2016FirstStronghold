package frc492;

import java.sql.Struct;

import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;

public class AutoDrawbridge implements AutoStrategy {
	
	private static final String moduleName = "AutoDrawbridge";
	private Robot robot;
	private double distance;
	private TrcEvent driveEvent;
	private TrcEvent elevatorEvent;
	private TrcEvent armEvent;
	
	private TrcStateMachine sm;

	private enum AutonomousState
	{
		MOVING_CLOSE_TO_DRAWBRIDGE_AT_HIGH_SPEED_WHILE_RAISING_ELEVATOR_AND_ROTATING_ARMS,
		MOVING_FORWARD_SLOWLY_TO_POSITION_IN_FRONT_OF_DRAWBRIDGE,
		LOWERING_ARMS_OVER_DRABRIDE,
		MOVING_BACK_SLOWLY_WHILE_LOWERING_ELEVATOR_SLOWLY_AND_ROTATING_ARMS_BACK_AT_A_SLOW_RATE,
		ADVANCING_THROUGH_DRAWBRIDGE_WHILE_LIFTING_ARMS
		
	}
	private AutonomousState State;
	
	public AutoDrawbridge(Robot robot)
	{
		this.robot = robot;
		
	}
	
	private static final float HIGH_SPEED_MOVING_POWER = 0.95f;
	private static final float MEDIUM_SPEED_MOVING_POWER = 0.5f;
	private static final int DISTANCE_TO_DRAWBRIDGE = 96;
	
	public void autoPeriodic(double elapsedTime) 
	{
		boolean ready = sm.isReady();
		
		if(!ready)
			return;

		switch(State)
		{
		case MOVING_CLOSE_TO_DRAWBRIDGE_AT_HIGH_SPEED_WHILE_RAISING_ELEVATOR_AND_ROTATING_ARMS:
			robot.encoderYPidCtrl.setOutputRange(-HIGH_SPEED_MOVING_POWER, HIGH_SPEED_MOVING_POWER);
			robot.pidDrive.setTarget(DISTANCE_TO_DRAWBRIDGE * (5/6), 0, false, driveEvent, 2);
			sm.addEvent(driveEvent);
			// TODO Add elevator and arm raising/turning
			break;
			
		case MOVING_FORWARD_SLOWLY_TO_POSITION_IN_FRONT_OF_DRAWBRIDGE:
			robot.encoderYPidCtrl.setOutputRange(-MEDIUM_SPEED_MOVING_POWER, MEDIUM_SPEED_MOVING_POWER);
			robot.pidDrive.setTarget(DISTANCE_TO_DRAWBRIDGE * (1/6), 0, true, driveEvent, 2);
			sm.addEvent(driveEvent);
			break;
			
		case LOWERING_ARMS_OVER_DRABRIDE:
			break;
			
		case MOVING_BACK_SLOWLY_WHILE_LOWERING_ELEVATOR_SLOWLY_AND_ROTATING_ARMS_BACK_AT_A_SLOW_RATE:
			break;
			
		case ADVANCING_THROUGH_DRAWBRIDGE_WHILE_LIFTING_ARMS:
			break;
				
		}
		
	}
	
}
