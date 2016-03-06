package frc492;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import frclib.FrcCANTalon;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTaskMgr.TaskType;
import trclib.TrcTimer;

public class Crane implements TrcPidController.PidInput, TrcTaskMgr.Task
{		
    private static final String moduleName = "Crane";

    private enum State
    {
        START,
        CRANE_UP,
        CHECK_CRANE,
        DONE
    }

    private HalDashboard dashboard = HalDashboard.getInstance();

    private FrcCANTalon winchMotor;
    private TrcPidController winchPidCtrl;
    private TrcPidMotor winchPidMotor;

    private FrcCANTalon craneMotor;
    private TrcPidController cranePidCtrl;
    private TrcPidMotor cranePidMotor;

    private FrcCANTalon tilterMotor;
    private TrcPidController tilterPidCtrl;
    private TrcPidMotor tilterPidMotor;

    private TrcStateMachine sm;
    private TrcTimer timer;
    private TrcEvent event;

    /**
     * Constructor: Create an instance of the object.
     */
    public Crane()
    {
        //
        // Winch has a motor and an encoder but no limit switches.
        // The encoder is used to synchronize with the crane motor.
        //
        winchMotor = new FrcCANTalon(RobotInfo.CANID_WINCH);
        winchMotor.setInverted(true);
        winchMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        winchMotor.reverseSensor(false);
        winchPidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.WINCH_KP,
                RobotInfo.WINCH_KI,
                RobotInfo.WINCH_KD,
                RobotInfo.WINCH_KF,
                RobotInfo.WINCH_TOLERANCE,
                RobotInfo.WINCH_SETTLING,
                this);
        winchPidMotor = new TrcPidMotor(moduleName + ".winch", winchMotor, winchPidCtrl);
        winchPidMotor.setPositionScale(RobotInfo.WINCH_INCHES_PER_COUNT);

        //
        // Crane has a motor, an encoder, lower and upper limit switches.
        // It can do full PID control.
        //
        craneMotor = new FrcCANTalon(RobotInfo.CANID_CRANE);
        craneMotor.setInverted(true);
        craneMotor.ConfigRevLimitSwitchNormallyOpen(true);
        craneMotor.ConfigFwdLimitSwitchNormallyOpen(true);
        craneMotor.setLimitSwitchesSwapped(true);
        craneMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        craneMotor.reverseSensor(true);
        cranePidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.CRANE_KP,
                RobotInfo.CRANE_KI,
                RobotInfo.CRANE_KD,
                RobotInfo.CRANE_KF,
                RobotInfo.CRANE_TOLERANCE,
                RobotInfo.CRANE_SETTLING,
                this);
        cranePidCtrl.setAbsoluteSetPoint(true);
        cranePidMotor = new TrcPidMotor(moduleName + ".crane", craneMotor, cranePidCtrl);
        /*
        cranePidMotor = new TrcPidMotor(
                moduleName + ".crane",
                craneMotor, winchMotor,
                RobotInfo.CRANE_SYNC_GAIN, cranePidCtrl);
        */
        //??? What to do about the encoder scale difference between crane and winch motors???
        cranePidMotor.setPositionScale(RobotInfo.CRANE_INCHES_PER_COUNT);

        //
        // Tilter has a motor, an encoder and a lower limit switch.
        // It can do full PID control.
        //
        tilterMotor = new FrcCANTalon(RobotInfo.CANID_TILTER);
        tilterMotor.setInverted(true);
        tilterMotor.ConfigRevLimitSwitchNormallyOpen(false);
        tilterMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        tilterMotor.setLimitSwitchesSwapped(true);
        tilterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        tilterMotor.reverseSensor(false);
        tilterPidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.TILTER_KP,
                RobotInfo.TILTER_KI,
                RobotInfo.TILTER_KD,
                RobotInfo.TILTER_KF,
                RobotInfo.TILTER_TOLERANCE,
                RobotInfo.TILTER_SETTLING,
                this);
        tilterPidCtrl.setAbsoluteSetPoint(true);
        tilterPidCtrl.setOutputRange(-0.5, 0.5);
        tilterPidMotor = new TrcPidMotor(
                moduleName + ".tilter", tilterMotor, tilterPidCtrl);
        tilterPidMotor.setPositionScale(RobotInfo.TILTER_DEGREES_PER_COUNT);

        sm = new TrcStateMachine(moduleName);
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
    }

    public void displayDebugInfo(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum, "Tilter: Angle=%.2f, SW=%d/%d",
                getTilterAngle(),
                tilterMotor.isLowerLimitSwitchActive()? 1: 0,
                tilterMotor.isUpperLimitSwitchActive()? 1: 0);
        tilterPidCtrl.displayPidInfo(lineNum + 1);
        dashboard.displayPrintf(
                lineNum + 3, "Crane: Length=%.2f, SW=%d/%d",
                getCraneLength(),
                craneMotor.isLowerLimitSwitchActive()? 1: 0,
                craneMotor.isUpperLimitSwitchActive()? 1: 0);
        cranePidCtrl.displayPidInfo(lineNum + 4);
        dashboard.displayPrintf(lineNum + 6, "Winch: Length=%.2f", getWinchLength());
    }

    /*
     * Zero Calibration
     */
    public void zeroCalibarateCrane()
    {
        cranePidMotor.zeroCalibrate(RobotInfo.CRANE_CAL_POWER);
    }

    public void zeroCalibrateTilter()
    {
        tilterPidMotor.zeroCalibrate(RobotInfo.TILTER_CAL_POWER);
    }

    /*
     * Set power
     */
    public void setCranePower(double power)
    {
        craneMotor.setPower(power);
//        cranePidMotor.setPower(power);
    }

    public void setWinchPower(double power)
    {
        winchMotor.setPower(power);
    }

    public void setTilterPower(double power)
    {
        tilterPidMotor.setPower(power);
    }

    /*
     * Set height or angle
     */
    public void setCraneLength(double length)
    {
        cranePidMotor.setTarget(length, true);
    }

    public void setCraneLength(double length, TrcEvent event, double timeout)
    {
        cranePidMotor.setTarget(length, event, timeout);
    }

    public void setTilterAngle(double angle)
    {
        tilterPidMotor.setTarget(angle, true);
    }

    public void setTilterAngle(double angle, TrcEvent event, double timeout)
    {
        tilterPidMotor.setTarget(angle, event, timeout);
    }

    /*
     * Accessors: Crane length, Tilter angle, limit switches
     */
    public double getCraneLength()
    {
        return cranePidMotor.getPosition();
    }

    public double getWinchLength()
    {
        return winchMotor.getPosition()*RobotInfo.WINCH_INCHES_PER_COUNT;
    }

    public double getTilterAngle()
    {
        return tilterPidMotor.getPosition();
    }

    public boolean isCraneLowerLimitSwitchActive()
    {
        return craneMotor.isLowerLimitSwitchActive();
    }

    public boolean isCraneUpperLimitSwitchActive()
    {
        return craneMotor.isUpperLimitSwitchActive();
    }

    public boolean isTilterLowerLimitSwitchActive()
    {
        return tilterMotor.isLowerLimitSwitchActive();
    }

    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(
                    "hangSequence", this, TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TaskType.POSTCONTINUOUS_TASK);
        }
    }

    public void hangSequence()
    {
        sm.start(State.START);
        setTaskEnabled(true);
    }

    /*
     * Implements TrcPidController.PidInput
     */
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.cranePidCtrl)
        {
            value = getCraneLength();
        }
        else if (pidCtrl == this.tilterPidCtrl)
        {
            value = getTilterAngle();
        }

        return value;
    }

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case START:
                    setTilterAngle(70.0);
                    winchMotor.setPower(1.0);
                    timer.set(4.0, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.CRANE_UP);
                    break;

                case CRANE_UP:
                    setCranePower(1.0);
                    sm.setState(State.CHECK_CRANE);
                    break;

                case CHECK_CRANE:
                    if (!craneMotor.isUpperLimitSwitchActive())
                    {
                        winchMotor.setPower(0.0);
                        sm.setState(State.DONE);
                    }
                    break;

                default:
                case DONE:
                    sm.stop();
                    setTaskEnabled(false);
                    break;
            }
        }
    }   //postContinuousTask

}   //class Crane
