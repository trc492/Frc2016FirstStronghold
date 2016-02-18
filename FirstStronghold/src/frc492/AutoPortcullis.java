package frc492;

//import frclibj.TrcDashboard;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;

/*
Autonomous for portcullis
rebecca cheng~~
 */

public class AutoPortcullis implements AutoStrategy
{
	// constants
    public static final double DISTANCE_TO_DEFENSE	= 50;
    public static final double DISTANCE_TO_PORT 	= 20;
    public static final double ARM_TO_NEUTRAL		= -1.0;
    public static final double ELEVATOR_TO_NEUTRAL 	= 1.0;	
	
	// variables
	private static final String moduleName = "AutoBinSet";
	//private HalDashboard dashboard = HalDashboard.getInstance();
	private Robot robot;
	private TrcEvent driveEvent;
	private TrcEvent elevatorEvent;
	private TrcEvent armEvent;
	private TrcStateMachine sm;
	
	// state machine
	private enum State
	{
		START,
		STATE_DRIVE_TO_DEFENSE,
		STATE_LOWER_ARMS,
		STATE_DRIVE_TO_PORT,
		STATE_LIFT_GATE,
		STATE_DRIVE_UNDER_PORT,
		STATE_ARMS_TO_NEUTRAL,
		STATE_DONE
	}
	
	// constructor takes: robot, distance from neutral line
	public AutoPortcullis (Robot robot)
	{
		this.robot = robot;
		driveEvent = new TrcEvent(moduleName + ".driveEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        armEvent = new TrcEvent(moduleName + ".armEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start(robot);
	}
	
	public void autoPeriodic(double elapsedTime)
	{
        boolean ready = sm.isReady();
        /*TrcDashboard.textPrintf(1, "%s[%d] = %s",
                moduleName,
                sm.getState(),
                ready? "Ready": "NotReady");*/

        if (ready)
        {
            State state = (State) sm.getState();
            switch (state)
            {
            case STATE_DRIVE_TO_DEFENSE:
                //
                // drive up to the defense
                //
            	robot.pidDrive.setTarget(DISTANCE_TO_DEFENSE, 0, false, driveEvent, 2.0);
            	robot.encoderYPidCtrl.setOutputRange(-1, 1);
            	
                sm.addEvent(driveEvent);
                sm.waitForEvents(State.STATE_LOWER_ARMS, 0.0, true);
                break;

            case STATE_LOWER_ARMS:
            	//
            	// lower elevator and lower arms
            	//
            	
            	if (robot.arm.getPosition() != 0)
            	{
            		robot.arm.setPosition(0, armEvent, 1.0);
            	}
            	
            	if (robot.elevator.getHeight() != 0)
                {
                    robot.elevator.setHeight(0, elevatorEvent, 1.0);
                }
                                
                sm.addEvent(armEvent);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(State.STATE_DRIVE_TO_PORT, 0.0, true);
                break;

            case STATE_DRIVE_TO_PORT:
                //
                // drive to the portcullius
                //
            	robot.pidDrive.setTarget(DISTANCE_TO_PORT, 0, false, driveEvent, 2.0);
            	robot.encoderYPidCtrl.setOutputRange(-.3, .3);
                sm.addEvent(driveEvent);
                sm.waitForEvents(State.STATE_LIFT_GATE, 0.0, true);
                break;
                
            case STATE_LIFT_GATE:
            	//
            	// lift the gate
            	//
            	robot.elevator.setHeight(RobotInfo.ELEVATOR_MAX_HEIGHT, elevatorEvent, 1.0);
            	robot.arm.setPosition(RobotInfo.ARM_MAX_POSITION, armEvent, 1.0);
            	
            	sm.addEvent(elevatorEvent);
                sm.addEvent(armEvent);
            	sm.waitForEvents(State.STATE_DRIVE_UNDER_PORT, 0.0, true);
                break;
            
            case STATE_DRIVE_UNDER_PORT:
            	//
            	// lift the gate
            	//
            	robot.pidDrive.setTarget(DISTANCE_TO_PORT, 0, false, driveEvent, 2.0);
            	robot.encoderYPidCtrl.setOutputRange(-.5, .5);
            	
            	sm.addEvent(driveEvent);
            	sm.waitForEvents(State.STATE_ARMS_TO_NEUTRAL, 0.0, true);
                break;
            
            case STATE_ARMS_TO_NEUTRAL:
            	//
            	// put arms into neutral position
            	//
            	robot.arm.setPosition(ARM_TO_NEUTRAL, armEvent, 1.0);
            	robot.elevator.setHeight(ELEVATOR_TO_NEUTRAL, elevatorEvent, 1.0);
            	
            	sm.addEvent(armEvent);
            	sm.addEvent(elevatorEvent);
            	sm.waitForEvents(State.STATE_DONE, 0.0, true);
            	

            case STATE_DONE:
            default:
                //
                // stop (in the name of love)
                //
                sm.stop();
            }
        }
    }   //autoPeriodic
}
