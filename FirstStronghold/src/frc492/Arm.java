package frc492;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import frclib.FrcCANTalon;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;

public class Arm implements TrcPidController.PidInput
{
    private static final String moduleName = "Arm";
    private FrcCANTalon leftMotor;
    private FrcCANTalon rightMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;

    public Arm()
    {
        leftMotor = new FrcCANTalon(RobotInfo.CANID_LEFT_ARM);
        rightMotor = new FrcCANTalon(RobotInfo.CANID_RIGHT_ARM);
        leftMotor.setInverted(true);
        rightMotor.setInverted(true);
        leftMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        rightMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        leftMotor.setLimitSwitchesSwapped(true);
        rightMotor.setLimitSwitchesSwapped(true);
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
        pidMotor.setPositionScale(RobotInfo.ARM_COUNTS_PER_DEGREE);
    }

    public void displayDebugInfo(int lineNum)
    {
        pidCtrl.displayPidInfo(lineNum);
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
        double leftPos = leftMotor.getPosition();
        double rightPos = rightMotor.getPosition();
        return (leftPos + rightPos)/RobotInfo.ARM_COUNTS_PER_DEGREE/2.0;
    }

    public void zeroCalibrate()
    {
        pidMotor.zeroCalibrate(RobotInfo.ARM_CAL_POWER);
    }

    public void setPower(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        /*
        pidMotor.setPidPower(
                power,
                RobotInfo.ARM_UP_POSITION,
                RobotInfo.ARM_DOWN_POSITION,
                true);
                */
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

}   //class Arm
