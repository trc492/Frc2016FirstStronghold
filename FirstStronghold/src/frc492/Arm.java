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
    private FrcCANTalon armMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;
    private TrcTimer timer;

    public Arm()
    {
        armMotor = new FrcCANTalon(RobotInfo.CANID_ARM);

        //Invert motor direction: arm should go down on positive power value.
        armMotor.setInverted(false);

        //Invert encoder: encode value should increase while arm going down.
        armMotor.setPositionSensorInverted(false);

        armMotor.enableLimitSwitch(true, true);
        armMotor.ConfigRevLimitSwitchNormallyOpen(false);
        armMotor.ConfigFwdLimitSwitchNormallyOpen(false);

        //Swap the two limit switches: lower limit switch should stop arm going down.
        armMotor.setLimitSwitchesSwapped(false);

        armMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);

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

        pidMotor = new TrcPidMotor(moduleName, armMotor, pidCtrl);

        //Need to determine degrees per encoder count
        pidMotor.setPositionScale(RobotInfo.ARM_DEGREES_PER_COUNT);

        timer = new TrcTimer(moduleName);
    }

    public void setLimitSwitchesEnabled(boolean enabled)
    {
        if (enabled)
        {
            armMotor.enableLimitSwitch (true, true);
            armMotor.ConfigRevLimitSwitchNormallyOpen(false);
            armMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        }
        else
        {
            armMotor.enableLimitSwitch (false, false);
        }
    }

    public void traceDebugInfo(TrcDbgTrace tracer)
    {
        pidCtrl.printPidInfo(tracer);
    }

    public void displayDebugInfo(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum, "Arm: Pos=%.2f",
                armMotor.getPosition());
        dashboard.displayPrintf(
                lineNum + 1, "Arm: SW=%d/%d",
                armMotor.isLowerLimitSwitchActive()? 1: 0,
                armMotor.isUpperLimitSwitchActive()? 1: 0);
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

    public double getRawPosition()
    {
        return armMotor.getPosition();
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
                    moduleName, "Arm: power=%.2f, Pos=%.0f",
                    power, armMotor.getPosition());
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
//        pidMotor.setPower(0.0);
        armMotor.set(0.0);
    }   //startTask

}   //class Arm
