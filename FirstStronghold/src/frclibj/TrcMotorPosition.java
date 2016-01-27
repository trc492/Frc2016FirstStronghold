package frclibj;

import edu.wpi.first.wpilibj.SpeedController;

public interface TrcMotorPosition
{
    public double getMotorPosition(SpeedController speedController);
    public double getMotorSpeed(SpeedController speedController);
    public void resetMotorPosition(SpeedController speedController);
    public void reversePositionSensor(
            SpeedController speedController,
            boolean flip);
    public boolean isForwardLimitSwitchActive(SpeedController speedController);
    public boolean isReverseLimitSwitchActive(SpeedController speedController);
}   //interface TrcMotorPosition
