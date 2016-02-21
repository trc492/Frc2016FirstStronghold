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
        AUTOMODE_PORTCULLIS,
        AUTOMODE_TUNE_ROBOT
    }

    public enum TuneMode
    {
        TUNEMODE_TIMED_DRIVE,
        TUNEMODE_MOVE_X,
        TUNEMODE_MOVE_Y,
        TUNEMODE_TURN,
        TUNEMODE_SONAR
    }

    private HalDashboard dashboard = HalDashboard.getInstance();
    private Robot robot;
    private SendableChooser autoChooser;
    private TrcRobot.AutoStrategy autoStrategy;
    private SendableChooser tuneChooser;

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
        autoChooser.addObject("Tune robot", AutoMode.AUTOMODE_TUNE_ROBOT);
        HalDashboard.putData("Autonomous Strategies", autoChooser);

        tuneChooser = new SendableChooser();
        tuneChooser.addDefault("Drive for 8 sec", TuneMode.TUNEMODE_TIMED_DRIVE);
        tuneChooser.addObject("Move X 20 ft", TuneMode.TUNEMODE_MOVE_X);
        tuneChooser.addObject("Move Y 20 ft", TuneMode.TUNEMODE_MOVE_Y);
        tuneChooser.addObject("Turn 360", TuneMode.TUNEMODE_TURN);
        tuneChooser.addObject("Sonar drive 7 in", TuneMode.TUNEMODE_SONAR);
        HalDashboard.putData("Robot tune modes", tuneChooser);
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
                dashboard.displayPrintf(1, "NoAuto");
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
    
            case AUTOMODE_TUNE_ROBOT:
                TuneMode selectedTuneMode = (TuneMode)(tuneChooser.getSelected());
                autoStrategy = new AutoTuneRobot(robot, selectedTuneMode);
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
