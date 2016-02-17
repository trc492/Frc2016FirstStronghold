package frc492;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class Autonomous implements TrcRobot.RobotMode
{
 	public enum AutoMode
    {
        AUTOMODE_NONE,
        AUTOMODE_TUNE_ROBOT
    }

    public enum TuneMode
    {
        TUNEMODE_MOVE_X,
        TUNEMODE_MOVE_Y,
        TUNEMODE_TURN,
        TUNEMODE_SONAR
    }

    private Robot robot;
    private HalDashboard dashboard;
    private SendableChooser autoChooser;
    private TrcRobot.AutoStrategy autoStrategy;
    private SendableChooser tuneChooser;

    public Autonomous(Robot robot)
    {
        this.robot = robot;
        dashboard = HalDashboard.getInstance();
        autoChooser = new SendableChooser();
        autoChooser.addDefault("No autonomous", AutoMode.AUTOMODE_NONE);

        autoChooser.addObject("Tune robot", AutoMode.AUTOMODE_TUNE_ROBOT);
        HalDashboard.putData("Autonomous Strategies", autoChooser);

        tuneChooser = new SendableChooser();
        tuneChooser.addDefault("Move X 20 ft", TuneMode.TUNEMODE_MOVE_X);
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
        robot.elevator.zeroCalibrate();

        AutoMode selectedAutoMode = (AutoMode)(autoChooser.getSelected());
        switch (selectedAutoMode)
        {
        case AUTOMODE_NONE:
            autoStrategy = null;
            dashboard.displayPrintf(1, "NoAuto");
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
