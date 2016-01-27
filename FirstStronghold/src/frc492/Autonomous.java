package frc492;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frclibj.TrcDashboard;
import frclibj.TrcRobot;

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
    private SendableChooser autoChooser;
    private TrcRobot.AutoStrategy autoStrategy;
    private SendableChooser tuneChooser;

    public Autonomous(Robot robot)
    {
        this.robot = robot;
        autoChooser = new SendableChooser();
        autoChooser.addDefault("No autonomous", AutoMode.AUTOMODE_NONE);

        autoChooser.addObject("Tune robot", AutoMode.AUTOMODE_TUNE_ROBOT);
        TrcDashboard.putData("Autonomous Strategies", autoChooser);

        tuneChooser = new SendableChooser();
        tuneChooser.addDefault("Move X 20 ft", TuneMode.TUNEMODE_MOVE_X);
        tuneChooser.addObject("Move Y 20 ft", TuneMode.TUNEMODE_MOVE_Y);
        tuneChooser.addObject("Turn 360", TuneMode.TUNEMODE_TURN);
        tuneChooser.addObject("Sonar drive 7 in", TuneMode.TUNEMODE_SONAR);
        TrcDashboard.putData("Robot tune modes", tuneChooser);

     }   //Autonomous

    //
    // Implements TrcRobot.RunMode.
    //
    public void start()
    {
        //
        // Turn on brake mode and limit drive power to make autonomous drive
        // more precise.
        //
        robot.driveBase.setBrakeModeEnabled(true);
        robot.xPidCtrl.setOutputRange(
                -RobotInfo.X_RANGE_LIMIT, RobotInfo.X_RANGE_LIMIT);
        robot.yPidCtrl.setOutputRange(
                -RobotInfo.Y_RANGE_LIMIT, RobotInfo.Y_RANGE_LIMIT);
        robot.turnPidCtrl.setOutputRange(
                -RobotInfo.TURN_RANGE_LIMIT, RobotInfo.TURN_RANGE_LIMIT);
        robot.sonarPidCtrl.setOutputRange(
                -RobotInfo.SONAR_RANGE_LIMIT, RobotInfo.SONAR_RANGE_LIMIT);
        robot.elevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);

        AutoMode selectedAutoMode = (AutoMode)(autoChooser.getSelected());
        switch (selectedAutoMode)
        {
        case AUTOMODE_NONE:
            autoStrategy = null;
            TrcDashboard.textPrintf(1, "NoAuto");
            break;

        case AUTOMODE_TUNE_ROBOT:
            TuneMode selectedTuneMode = (TuneMode)(tuneChooser.getSelected());
            autoStrategy = new AutoTuneRobot(robot, selectedTuneMode);
            break;
        }
    }   //start

    public void stop()
    {
        robot.driveBase.stop();
    }   //stop

    public void periodic()
    {
        if (autoStrategy != null)
        {
            autoStrategy.autoPeriodic();
        }
        robot.updateDashboard();
    }   //periodic

    public void continuous()
    {
    }   //continuous

}   //class Autonomous
