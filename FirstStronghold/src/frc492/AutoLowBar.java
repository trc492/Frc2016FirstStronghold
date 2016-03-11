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
    private double distanceOverDefense;
    private double distanceToTower;
    private double turnToTower;
    private double distanceToGoal;
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;
    
    private enum State
    {
        TEST,
        DRIVE_TO_DEFENSE,
        DRIVE_OVER_DEFENSE,
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
                RobotInfo.AUTOKEY_DISTANCE_TO_DEFENSE, RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceOverDefense = HalDashboard.getNumber(
                RobotInfo.AUTOKEY_DISTANCE_OVER_DEFENSE, RobotInfo.AUTO_DISTANCE_OVER_DEFENSE);
        distanceToTower = HalDashboard.getNumber(
                RobotInfo.AUTOKEY_DISTANCE_TO_TOWER, RobotInfo.AUTO_DISTANCE_TO_TOWER);
        turnToTower = HalDashboard.getNumber(
                RobotInfo.AUTOKEY_TURN_TO_TOWER, RobotInfo.AUTO_TURN_TO_TOWER);
        distanceToGoal = HalDashboard.getNumber(
                RobotInfo.AUTOKEY_DISTANCE_TO_GOAL, RobotInfo.AUTO_DISTANCE_TO_GOAL);

        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine(moduleName);
        timer = new TrcTimer(moduleName);

        sm.start(State.DRIVE_TO_DEFENSE);
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
                case TEST:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, 96.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DRIVE_TO_DEFENSE:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToDefense, 0.0, false, event, 2.0);
                    robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_OVER_DEFENSE);
                    break;

                case DRIVE_OVER_DEFENSE:
                    robot.encoderYPidCtrl.setOutputRange(-0.4, 0.4);
                    robot.pidDrive.setTarget(0.0, distanceOverDefense, 0.0, false, event, 5.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_TO_TOWER);
                    break;

                case DRIVE_TO_TOWER:
                    robot.encoderYPidCtrl.setOutputRange(-0.7, 0.7);
                    robot.pidDrive.setTarget(0.0, distanceToTower, 0.0, false, event, 2.0);
                    robot.crane.setTilterAngle(20.0);
                    robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
                    sm.addEvent(event);
                    sm.waitForEvents(State.TURN_TO_TOWER);
                    break;

                case TURN_TO_TOWER:
                    robot.gyroTurnPidCtrl.setOutputRange(-0.7, 0.7);
                    robot.pidDrive.setTarget(0.0, 0.0, turnToTower, false, event, 1.0);
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
                    timer.set(5.0, event);
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
