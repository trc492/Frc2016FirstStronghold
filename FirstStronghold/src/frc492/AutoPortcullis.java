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
    public static final double ARM_PORT_LOW         = 0.0;
    public static final double ARM_PORT_UP          = 0.0;
    public static final double DISTANCE_UNDER_PORT  = 0.0;
    public static final double DISTANCE_PAST_PORT   = 0.0;
	
	// variables
	private static final String moduleName = "AutoPortcullis";
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
		STATE_DRIVE_TO_PORT,
		STATE_LIFT_GATE,
		STATE_DRIVE_UNDER_PORT,
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
        sm.start(State.START);
	}
	
	public void autoPeriodic(double elapsedTime)
	{
        /*TrcDashboard.textPrintf(1, "%s[%d] = %s",
                moduleName,
                sm.getState(),
                ready? "Ready": "NotReady");*/

        if (sm.isReady())
        {
            State state = (State) sm.getState();
            switch (state)
            {
            case STATE_DRIVE_TO_DEFENSE:
                //
                // drive up to the defense
                //
                robot.encoderYPidCtrl.setOutputRange(-1, 1);
            	robot.pidDrive.setTarget(0.0, DISTANCE_TO_DEFENSE, 0.0, false, driveEvent, 2.0);
            	
            	//
            	// lower elevator
            	//
                robot.elevator.setHeight(RobotInfo.ELEVATOR_MIN_HEIGHT);
                robot.arm.setPosition(ARM_PORT_LOW);
               
                sm.addEvent(driveEvent);
                sm.waitForEvents(State.STATE_DRIVE_TO_PORT, 0.0, true);
                break;

            case STATE_DRIVE_TO_PORT:
                //
                // drive to the portcullius
                //
                robot.encoderYPidCtrl.setOutputRange(-.3, .3);
            	robot.pidDrive.setTarget(0.0, DISTANCE_TO_PORT, 0.0, false, driveEvent, 2.0);
            	
                sm.addEvent(driveEvent);
                sm.waitForEvents(State.STATE_LIFT_GATE, 0.0, true);
                break;
                
            case STATE_LIFT_GATE:
            	//
            	// lift the gate
            	//
            	robot.elevator.setHeight(RobotInfo.ELEVATOR_MAX_HEIGHT, elevatorEvent, 1.0);
            	robot.arm.setPosition(ARM_PORT_UP, armEvent, 1.0);
            	
            	sm.addEvent(elevatorEvent);
                sm.addEvent(armEvent);
            	sm.waitForEvents(State.STATE_DRIVE_UNDER_PORT, 0.0, true);
                break;
            
            case STATE_DRIVE_UNDER_PORT:
            	//
            	// lift the gate
            	//
                robot.encoderYPidCtrl.setOutputRange(-.5, .5);
                robot.pidDrive.setTarget(0.0, DISTANCE_UNDER_PORT, 0, false, driveEvent, 2.0);
            	//robot.arm.setPosition(position);
                
            	sm.addEvent(driveEvent);
            	sm.waitForEvents(State.STATE_DONE, 0.0, true);
                break;
            	

            case STATE_DONE:
            default:
                //
                // stop (in the name of love)
                //
            	robot.pidDrive.setTarget(0.0, DISTANCE_PAST_PORT, 0, false, driveEvent, 2.0);
                robot.arm.setPosition(ARM_TO_NEUTRAL);
                robot.elevator.setHeight(ELEVATOR_TO_NEUTRAL);
                
                sm.stop();
            }
        }
    }   //autoPeriodic
}
