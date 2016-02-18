package frc492;

import java.sql.Struct;

import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;
import hallib.HalDashboard;

public class AutoDrawbridge implements AutoStrategy {
	
	private static final String moduleName = "AutoDrawbridge";
	private Robot robot;
	private TrcEvent driveEvent;
	private TrcEvent elevatorEvent;
	private TrcEvent armEvent;
	private TrcStateMachine sm;
	
	private enum AutonomousState
	{
		MOVING_CLOSE_TO_DRAWBRIDGE_AT_HIGH_SPEED_WHILE_RAISING_ELEVATOR_AND_ROTATING_ARMS,
		MOVING_FORWARD_SLOWLY_TO_POSITION_IN_FRONT_OF_DRAWBRIDGE,
		LOWERING_ARMS_OVER_DRAWBRIDGE,
		MOVING_BACK_SLOWLY_WHILE_LOWERING_ELEVATOR_SLOWLY_AND_ROTATING_ARMS_BACK_AT_A_SLOW_RATE,
		ADVANCING_THROUGH_DRAWBRIDGE_WHILE_LIFTING_ARMS,
		STOP
	}

	public AutoDrawbridge(Robot robot)
	{
		this.robot = robot;
	    driveEvent = new TrcEvent(moduleName + ".drive");
	    elevatorEvent = new TrcEvent(moduleName + ".elevator");
	    armEvent = new TrcEvent(moduleName + ".arm");
	    sm = new TrcStateMachine(moduleName);
	    sm.start(AutonomousState.MOVING_CLOSE_TO_DRAWBRIDGE_AT_HIGH_SPEED_WHILE_RAISING_ELEVATOR_AND_ROTATING_ARMS);
	}
	
	private static final double HIGH_SPEED_MOVING_POWER = 0.95;
	private static final double MEDIUM_SPEED_MOVING_POWER = 0.5;
	private static final double DISTANCE_TO_DRAWBRIDGE = 96.0;
	private static final double ARM_CLOSED_DRAWBRIDGE_POSITION = 30.0;
	private static final double ELEVATOR_OPEN_DRAWBRIDGE_POSITION = 10.0;
	private static final double ARM_OPEN_DRAWBRIDGE_POSITION = 5.0;
	private static final double ARM_MID_POSITION = 20.0;
	
	public void autoPeriodic(double elapsedTime) 
	{
		if (!sm.isReady())
			return;

        AutonomousState state = (AutonomousState)sm.getState();
        
		switch(state)
		{
		case MOVING_CLOSE_TO_DRAWBRIDGE_AT_HIGH_SPEED_WHILE_RAISING_ELEVATOR_AND_ROTATING_ARMS:
			robot.encoderYPidCtrl.setOutputRange(-HIGH_SPEED_MOVING_POWER, HIGH_SPEED_MOVING_POWER);
			robot.pidDrive.setTarget(0.0, DISTANCE_TO_DRAWBRIDGE * (5/6), 0, false, driveEvent, 2.0);
			sm.addEvent(driveEvent);
			robot.elevator.setHeight(RobotInfo.ELEVATOR_MAX_HEIGHT, elevatorEvent, 0.0);
//			sm.addEvent(elevatorEvent);
			robot.arm.setPosition(RobotInfo.ARM_UP_POSITION, armEvent, 0.0);
//			sm.addEvent(armEvent);
			sm.waitForEvents(AutonomousState.MOVING_FORWARD_SLOWLY_TO_POSITION_IN_FRONT_OF_DRAWBRIDGE);
			break;
			
		case MOVING_FORWARD_SLOWLY_TO_POSITION_IN_FRONT_OF_DRAWBRIDGE:
			robot.encoderYPidCtrl.setOutputRange(-MEDIUM_SPEED_MOVING_POWER, MEDIUM_SPEED_MOVING_POWER);
			robot.pidDrive.setTarget(0.0, DISTANCE_TO_DRAWBRIDGE * (1/6), 0.0, true, driveEvent, 2);
			sm.addEvent(driveEvent);
			sm.waitForEvents(AutonomousState.LOWERING_ARMS_OVER_DRAWBRIDGE);
			break;
			
		case LOWERING_ARMS_OVER_DRAWBRIDGE:
			robot.arm.setPosition(ARM_CLOSED_DRAWBRIDGE_POSITION, armEvent, 0.0);
			sm.addEvent(armEvent);
			sm.waitForEvents(AutonomousState.MOVING_BACK_SLOWLY_WHILE_LOWERING_ELEVATOR_SLOWLY_AND_ROTATING_ARMS_BACK_AT_A_SLOW_RATE);
			break;
			
		case MOVING_BACK_SLOWLY_WHILE_LOWERING_ELEVATOR_SLOWLY_AND_ROTATING_ARMS_BACK_AT_A_SLOW_RATE:
			robot.encoderYPidCtrl.setOutputRange(-0.2, 0.2);
			robot.pidDrive.setTarget(0.0, -24.0, 0.0, false, driveEvent,  3.0);
			sm.addEvent(driveEvent);
			robot.elevator.setHeight(ELEVATOR_OPEN_DRAWBRIDGE_POSITION, elevatorEvent, 0.0);
//			sm.addEvent(elevatorEvent);
			robot.arm.setPosition(ARM_OPEN_DRAWBRIDGE_POSITION, armEvent, 0.0);
//			sm.addEvent(armEvent);
			sm.waitForEvents(AutonomousState.ADVANCING_THROUGH_DRAWBRIDGE_WHILE_LIFTING_ARMS);
			break;
			
		case ADVANCING_THROUGH_DRAWBRIDGE_WHILE_LIFTING_ARMS:
			robot.pidDrive.setTarget(0.0, 60.0, 0.0,false, driveEvent, 2.0);
			sm.addEvent(driveEvent);
			robot.arm.setPosition(ARM_MID_POSITION, armEvent, 0.0);
//			sm.addEvent(armEvent);
			sm.waitForEvents(AutonomousState.STOP);
			break;
		case STOP:
	    default:
			sm.stop();
		}
	}
}
