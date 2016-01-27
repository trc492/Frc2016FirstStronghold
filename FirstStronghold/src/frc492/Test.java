package frc492;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frclibj.TrcRobot;

public class Test implements TrcRobot.RobotMode
{
    //
    // Implements TrcRobot.RobotMode.
    //
    public void start()
    {
    }   //start

    public void stop()
    {
    }   //stop

    public void periodic()
    {
        LiveWindow.run();
    }   //periodic

    public void continuous()
    {
    }   //continuous

}   //class Test
