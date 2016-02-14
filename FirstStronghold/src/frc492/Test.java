package frc492;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import trclib.TrcRobot;

public class Test implements TrcRobot.RobotMode
{
    //
    // Implements TrcRobot.RobotMode.
    //
    public void startMode()
    {
    }   //startMode

    public void stopMode()
    {
    }   //stopMode

    public void runPeriodic(double elapsedTime)
    {
        LiveWindow.run();
    }   //runPeriodic

    public void runContinuous(double elapsedTime)
    {
    }   //runContinuous

}   //class Test
