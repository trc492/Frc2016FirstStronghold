package frc492;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import frclib.FrcCANTalon;
import frclib.FrcRobotBase;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcTimer;

public class Arm implements TrcPidController.PidInput, TrcTimer.Callback
{
    private static final String moduleName = "Arm";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = debugEnabled? FrcRobotBase.getRobotTracer(): null;

    private HalDashboard dashboard = HalDashboard.getInstance();
    private FrcCANTalon leftMotor;
    private FrcCANTalon rightMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;
    private TrcTimer timer;

    public Arm()
    {
        leftMotor = new FrcCANTalon(RobotInfo.CANID_LEFT_ARM);
        rightMotor = new FrcCANTalon(RobotInfo.CANID_RIGHT_ARM);
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);
        leftMotor.setPositionSensorInverted(false);
        rightMotor.setPositionSensorInverted(false);
        leftMotor.enableLimitSwitch(true, true);
        leftMotor.ConfigRevLimitSwitchNormallyOpen(false);
        rightMotor.ConfigRevLimitSwitchNormallyOpen(false);
        leftMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        rightMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        leftMotor.setLimitSwitchesSwapped(false);
        rightMotor.setLimitSwitchesSwapped(false);
        leftMotor.setFeedbackDevice(FeedbackDevice.AnalogPot);
        rightMotor.setFeedbackDevice(FeedbackDevice.AnalogPot);

        pidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.ARM_KP,
                RobotInfo.ARM_KI,
                RobotInfo.ARM_KD,
                RobotInfo.ARM_KF,
                RobotInfo.ARM_TOLERANCE,
                RobotInfo.ARM_SETTLING,
                this);
        pidCtrl.setAbsoluteSetPoint(true);

        pidMotor = new TrcPidMotor(
                moduleName,
                leftMotor, rightMotor,
                RobotInfo.ARM_SYNC_GAIN,
                pidCtrl); 
        pidMotor.setPositionScale(RobotInfo.ARM_DEGREES_PER_COUNT);

        timer = new TrcTimer(moduleName);
    }

    public void setLimitSwitchesEnabled(boolean enabled)
    {
        if (enabled)
        {
            leftMotor.enableLimitSwitch (true, true);
            rightMotor.enableLimitSwitch(true, true);
            leftMotor.ConfigRevLimitSwitchNormallyOpen(false);
            rightMotor.ConfigRevLimitSwitchNormallyOpen(false);
            leftMotor.ConfigFwdLimitSwitchNormallyOpen(false);
            rightMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        }
        else
        {
            leftMotor.enableLimitSwitch (false, false);
            rightMotor.enableLimitSwitch(false, false);
        }
    }

    public void traceDebugInfo(TrcDbgTrace tracer)
    {
        pidCtrl.printPidInfo(tracer);
    }

    public void displayDebugInfo(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum, "Arm: lPos=%.2f, rPos=%.2f",
                leftMotor.getPosition(), rightMotor.getPosition());
        dashboard.displayPrintf(
                lineNum + 1, "Arm: lSW=%d/%d, rSW=%d/%d",
                leftMotor.isLowerLimitSwitchActive()? 1: 0,
                leftMotor.isUpperLimitSwitchActive()? 1: 0,
                rightMotor.isLowerLimitSwitchActive()? 1: 0,
                rightMotor.isUpperLimitSwitchActive()? 1: 0);
        pidCtrl.displayPidInfo(lineNum + 2);
    }

    public void setPosition(double position)
    {
        pidMotor.setTarget(position, true);
    }

    public void setPosition(double position, TrcEvent event, double timeout)
    {
        pidMotor.setTarget(position, event, timeout);
    }

    public double getPosition()
    {
        return pidMotor.getPosition();
    }

    public double getLeftRawPosition()
    {
        return leftMotor.getPosition();
    }

    public double getRightRawPosition()
    {
        return leftMotor.getPosition();
    }

    public void zeroCalibrate()
    {
        pidMotor.zeroCalibrate(RobotInfo.ARM_CAL_POWER);
    }

    public void setPower(double power, double time)
    {
        pidMotor.setPower(power, false);
        timer.set(time, this);
    }

    public void setPower(double power, boolean syncEnabled)
    {
        if (debugEnabled)
        {
            dbgTrace.traceInfo(
                    moduleName, "Arm: power=%.2f, lPos=%.0f, rPos=%.0f",
                    power, leftMotor.getPosition(), rightMotor.getPosition());
        }

        //
        // Don't synchronize the motors if in manual override mode. This is useful if the
        // encoder or limit switches are malfunctioning. Synchronization needs the lower
        // limit switch and encoder. So if any of these are malfunctioning, we need a way
        // to still control the arm in a reasonable fashion.
        //
        pidMotor.setPower(power, false);
    }

    //
    // Implements TrcPidController.PidInput.
    //

    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.pidCtrl)
        {
            value = getPosition();
        }

        return value;
    }   //getInput

    //
    // Implements TrcTimer.Callback
    //

    @Override
    public void timerCallback(TrcTimer timer, boolean canceled)
    {
        pidMotor.setPower(0.0);
    }   //startTask

}   //class Arm
