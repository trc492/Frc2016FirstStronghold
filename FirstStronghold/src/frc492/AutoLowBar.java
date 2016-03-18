package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class AutoLowBar implements TrcRobot.AutoStrategy
{
    private static final String moduleName = "AutoLowBar";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private Robot robot;
    private double distanceToDefense;
    private double distanceCrossDefense;
    private double distanceToTower;
    private double turnToTower;
    private double distanceToGoal;
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;
    
    private enum State
    {
        DRIVE_TO_LOWBAR,
        CROSS_LOWBAR,
        DRIVE_TO_TOWER,
        TURN_TO_TOWER,
        DRIVE_TO_GOAL,
        SCORE_GOAL,
        DONE
    }

    public AutoLowBar(Robot robot)
    {
        this.robot = robot;
        distanceToDefense = HalDashboard.getNumber(
                "1.LowBar:DistanceToDefense", RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceCrossDefense = HalDashboard.getNumber(
                "2.LowBar:DistanceCrossDefense", RobotInfo.AUTO_DISTANCE_CROSS_DEFENSE);
        distanceToTower = HalDashboard.getNumber("3.LowBar:DistanceToTower", 200.0);
        turnToTower = HalDashboard.getNumber("4.LowBar:TurnToTower", 70.0);
        distanceToGoal = HalDashboard.getNumber("5.LowBar:DistanceToGoal", 118.0);

        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm.start(State.DRIVE_TO_LOWBAR);
     }   //Autonomous

    //
    // Implements TrcRobot.AutoStrategy.
    //
    @Override
    public void autoPeriodic(double elapsedTime)
    {
        State state = (State)sm.getState();
        dashboard.displayPrintf(
                1, "[%6.3f] %s", elapsedTime, state != null? state.toString(): "DISABLED");
        robot.encoderXPidCtrl.displayPidInfo(2);
        robot.encoderYPidCtrl.displayPidInfo(4);
        robot.gyroTurnPidCtrl.displayPidInfo(6);
        robot.arm.displayDebugInfo(8);

        if (sm.isReady())
        {
            state = (State)sm.getState();

            switch (state)
            {
                case DRIVE_TO_LOWBAR:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToDefense, 0.0, false, event, 2.0);
//                    robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);
                    robot.arm.setPower(1.0, 1.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.CROSS_LOWBAR);
                    break;

                case CROSS_LOWBAR:
                    robot.encoderYPidCtrl.setOutputRange(-0.4, 0.4);
                    robot.pidDrive.setTarget(0.0, distanceCrossDefense, 0.0, false, event, 3.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_TO_TOWER);
                    break;

                case DRIVE_TO_TOWER:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToTower, 0.0, false, event, 3.0);
                    robot.crane.setTilterAngle(20.0);
//                    robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
                    robot.arm.setPower(-1.0, 1.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.TURN_TO_TOWER);
                    break;

                case TURN_TO_TOWER:
                    robot.gyroTurnPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, 0.0, turnToTower, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_TO_GOAL);
                    break;

                case DRIVE_TO_GOAL:
                    robot.driveBase.resetPosition();
                    robot.pidDrive.setTarget(0.0, distanceToGoal, 0.0, false, event, 3.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.SCORE_GOAL);
                    break;

                case SCORE_GOAL:
                    robot.pickup.setPower(RobotInfo.PICKUP_OUT_POWER);
                    timer.set(2.0, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    robot.pickup.setPower(0.0);
                    sm.stop();
                    break;
            }
      }
    }   //autoPeriodic

}   //class AutoLowBar
