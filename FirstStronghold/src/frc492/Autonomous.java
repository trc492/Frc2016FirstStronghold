package frc492;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class Autonomous implements TrcRobot.RobotMode
{
    public enum AutoMode
    {
        AUTOMODE_NONE,
        AUTOMODE_LOW_BAR,
        AUTOMODE_ROUGH_TERRAIN,
        AUTOMODE_MOAT,
        AUTOMODE_RAMPARTS,
        AUTOMODE_ROCK_WALL,
        AUTOMODE_TEETER_TOTTER,
        AUTOMODE_PORTCULLIS
    }

    private HalDashboard dashboard = HalDashboard.getInstance();
    private Robot robot;
    private SendableChooser autoChooser;
    private TrcRobot.AutoStrategy autoStrategy;

    public Autonomous(Robot robot)
    {
        this.robot = robot;

        autoChooser = new SendableChooser();
        autoChooser.addDefault("No autonomous", AutoMode.AUTOMODE_NONE);
        autoChooser.addObject("Low bar", AutoMode.AUTOMODE_LOW_BAR);
        autoChooser.addObject("Rough terrain", AutoMode.AUTOMODE_ROUGH_TERRAIN);
        autoChooser.addObject("Moat", AutoMode.AUTOMODE_MOAT);
        autoChooser.addObject("Ramparts", AutoMode.AUTOMODE_RAMPARTS);
        autoChooser.addObject("Rock wall", AutoMode.AUTOMODE_ROCK_WALL);
        autoChooser.addObject("Teeter totter", AutoMode.AUTOMODE_TEETER_TOTTER);
        autoChooser.addObject("Portcullis", AutoMode.AUTOMODE_PORTCULLIS);
        HalDashboard.putData("Autonomous Strategies", autoChooser);
     }   //Autonomous

    //
    // Implements TrcRobot.RunMode.
    //

    public void startMode()
    {
        robot.arm.zeroCalibrate();

        AutoMode selectedAutoMode = (AutoMode)(autoChooser.getSelected());
        switch (selectedAutoMode)
        {
            default:
            case AUTOMODE_NONE:
                autoStrategy = null;
                break;

            case AUTOMODE_LOW_BAR:
                autoStrategy = new AutoLowBar(robot);
                break;

            case AUTOMODE_ROUGH_TERRAIN:
                autoStrategy = new AutoRoughTerrain(robot);
                break;

            case AUTOMODE_MOAT:
                autoStrategy = new AutoMoat(robot);
                break;

            case AUTOMODE_RAMPARTS:
                autoStrategy = new AutoRamparts(robot);
                break;

            case AUTOMODE_ROCK_WALL:
                autoStrategy = new AutoRockWall(robot);
                break;

            case AUTOMODE_TEETER_TOTTER:
                autoStrategy = new AutoTeeterTotter(robot);
                break;

            case AUTOMODE_PORTCULLIS:
                autoStrategy = new AutoPortcullis(robot);
                break;
        }
    }   //startMode

    public void stopMode()
    {
        robot.driveBase.stop();
    }   //stopMode

    public void runPeriodic(double elapsedTime)
    {
        robot.updateDashboard();
    }   //runPeriodic

    public void runContinuous(double elapsedTime)
    {
        if (autoStrategy != null)
        {
            autoStrategy.autoPeriodic(elapsedTime);
        }
    }   //runContinuous

}   //class Autonomous
