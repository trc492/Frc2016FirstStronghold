package frc492;

import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.vision.AxisCamera.Resolution;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcVision.ParticleReport;
import hallib.HalDashboard;
import trclib.TrcVision;

public class VisionTarget implements TrcTaskMgr.Task
{
    private static final String moduleName = "VisionTarget";
    private static final boolean debugEnabled = false;
    private static final boolean debugVision = false;
    private TrcDbgTrace dbgTrace = null;
    private HalDashboard dashboard = HalDashboard.getInstance();

    public class TargetReport
    {
        int imageWidth;
        int imageHeight;
        double boundingRectLeft;
        double boundingRectTop;
        double boundingRectRight;
        double boundingRectBottom;
        double areaScore;
        double aspectScore;
        double distance;
        boolean isTote;

        public String toString()
        {
            return  "\nimageWidth  = " + imageWidth +
                    "\nimageHeight = " + imageHeight +
                    "\nrectLeft    = " + boundingRectLeft +
                    "\nrectTop     = " + boundingRectTop +
                    "\nrectRight   = " + boundingRectRight +
                    "\nrectBottom  = " + boundingRectBottom +
                    "\nareaScore   = " + areaScore +
                    "\naspectScore = " + aspectScore +
                    "\ndistance    = " + distance + 
                    "\nisTote      = " + isTote;
        }   //toString

    }   //class TargetReport

    private final double AREA_MINIMUM = 0.5;    //Default min area as % total area
//    private final double LONG_RATIO = 0.22;     //Tote:LongSide/Height=26.9/12.1
//    private final double SHORT_RATIO = 1.4;     //Tote:ShortSide/Height=16.9/12.1
    private final double SCORE_MIN = 75.0;      //Min score to be a tote
    private final double VIEW_ANGLE = 64.0;     //View angle for Axis M1013
    private Relay ringLightPower;
    private AxisCamera camera;
    private Image image;
    private boolean hasImage;
    private Range[] colorThresholds;
    private ParticleFilterCriteria2[] filterCriteria;
    private ParticleFilterOptions2 filterOptions;
    private TargetReport targetReport;
    private TrcVision visionTask;

    public VisionTarget()
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        ringLightPower = new Relay(
                RobotInfo.RELAY_RINGLIGHT_POWER, Relay.Direction.kForward);
        camera = new AxisCamera("10.4.92.11");
        camera.writeResolution(Resolution.k320x240);
        camera.writeCompression(30);
        camera.writeBrightness(50);
        camera.writeMaxFPS(10);
        image = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
        hasImage = false;

        colorThresholds = new Range[3];
        colorThresholds[0] = new Range(101, 64);
        colorThresholds[1] = new Range(88, 255);
        colorThresholds[2] = new Range(134, 255);
        filterCriteria = new ParticleFilterCriteria2[1];
        filterCriteria[0] = new ParticleFilterCriteria2(
                MeasurementType.MT_AREA_BY_IMAGE_AREA,
                AREA_MINIMUM, 100.0,
                0, 0);
        filterOptions = new ParticleFilterOptions2(0, 0, 1, 1);
        targetReport = new TargetReport();
        visionTask = new TrcVision(
                camera,
                ImageType.IMAGE_RGB,
                ColorMode.HSV,
                colorThresholds,
                false,
                filterCriteria,
                filterOptions);
        if (debugVision)
        {
            setStreamingEnabled(true);
        }
//        visionTask.setImageFile("/home/lvuser/SampleImages/image.jpg");
//        visionTask.setSendImageEnabled(false);
    }   //VisionTarget

    public void setVisionTaskEnabled(boolean enabled)
    {
        final String funcName = "setVisionTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API);
        }
        setRingLightPowerOn(enabled);
        visionTask.setTaskEnabled(enabled);
    }   //setVisionTaskEnabled

    public void setRingLightPowerOn(boolean on)
    {
        final String funcName = "setRightLightPowerOn";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "on=%s", Boolean.toString(on));
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API);
        }
        ringLightPower.set(on? Relay.Value.kOn: Relay.Value.kOff);
    }   //setRightLightPowerOn

    public TargetReport getTargetReport()
    {
        final String funcName = "getTargetReport";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        Vector<ParticleReport> reports = visionTask.getTargets();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(
                    funcName, "reports=%s,numReports=%d",
                    reports != null? "Reports": "null",
                    reports != null? reports.size(): 0);
        }

        boolean hasReport = reports != null && reports.size() >= 1;
        if (hasReport)
        {
            double bestTarget1Score = 0.0;
            double bestTarget2Score = 0.0;
            int bestTarget1Index = -1;
            int bestTarget2Index = -1;

            hasImage = camera.getImage(image);
            for (int i = 0; i < reports.size(); i++)
            {
                double aspectScore = AspectScore(reports.elementAt(i));
                double areaScore = AreaScore(reports.elementAt(i));
                double combinedScore = aspectScore + areaScore;
                boolean isTarget =
                        aspectScore > SCORE_MIN && areaScore > SCORE_MIN;

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(
                            funcName,
                            "rect[%d] = {%3.0f,%3.0f/%3.0f,%3.0f}, " +
                            "aspect=%f, area=%f, isTarget=%s",
                            i,
                            reports.elementAt(i).boundingRectLeft,
                            reports.elementAt(i).boundingRectTop,
                            reports.elementAt(i).boundingRectRight,
                            reports.elementAt(i).boundingRectBottom,
                            aspectScore, areaScore,
                            Boolean.toString(isTarget));
                }

                if (hasImage)
                {
                    highLightTarget(
                            image,
                            reports.elementAt(i).boundingRectLeft,
                            reports.elementAt(i).boundingRectTop,
                            reports.elementAt(i).boundingRectRight,
                            reports.elementAt(i).boundingRectBottom);
                }

                if (bestTarget1Index == -1)
                {
                    bestTarget1Index = i;
                    bestTarget1Score = combinedScore;
                }
                else if (combinedScore > bestTarget1Score)
                {
                    bestTarget2Index = bestTarget1Index;
                    bestTarget2Score = bestTarget1Score;
                    bestTarget1Index = i;
                    bestTarget1Score = combinedScore;
                }
                else if (bestTarget2Index == -1 ||
                         combinedScore > bestTarget2Score)
                {
                    bestTarget2Index = i;
                    bestTarget2Score = combinedScore;
                }
            }

            targetReport.imageWidth =
                    reports.elementAt(bestTarget1Index).imageWidth;
            targetReport.imageHeight =
                    reports.elementAt(bestTarget1Index).imageHeight;
            if (bestTarget2Index == -1)
            {
                targetReport.boundingRectLeft =
                        reports.elementAt(bestTarget1Index).boundingRectLeft;
                targetReport.boundingRectRight =
                        reports.elementAt(bestTarget1Index).boundingRectRight;
                targetReport.boundingRectTop =
                        reports.elementAt(bestTarget1Index).boundingRectTop;
                targetReport.boundingRectBottom =
                        reports.elementAt(bestTarget1Index).boundingRectBottom;
                targetReport.aspectScore = AspectScore(reports.elementAt(bestTarget1Index));
                targetReport.areaScore = AreaScore(reports.elementAt(bestTarget1Index));
            }
            else
            {
                targetReport.boundingRectLeft =
                    Math.min(reports.elementAt(bestTarget1Index).boundingRectLeft,
                             reports.elementAt(bestTarget2Index).boundingRectLeft);
                targetReport.boundingRectRight =
                    Math.max(reports.elementAt(bestTarget1Index).boundingRectRight,
                             reports.elementAt(bestTarget2Index).boundingRectRight);
                targetReport.boundingRectTop =
                    Math.min(reports.elementAt(bestTarget1Index).boundingRectTop,
                             reports.elementAt(bestTarget2Index).boundingRectTop);
                targetReport.boundingRectBottom =
                    Math.max(reports.elementAt(bestTarget1Index).boundingRectBottom,
                             reports.elementAt(bestTarget2Index).boundingRectBottom);
                targetReport.aspectScore =
                        (AspectScore(reports.elementAt(bestTarget1Index)) +
                         AspectScore(reports.elementAt(bestTarget2Index)))/2;
                targetReport.areaScore =
                        (AreaScore(reports.elementAt(bestTarget1Index)) +
                         AreaScore(reports.elementAt(bestTarget2Index)))/2;
            }
            targetReport.distance =
                    computeDistance(
                            targetReport.boundingRectRight -
                            targetReport.boundingRectLeft,
                            targetReport.imageWidth);
            targetReport.isTote =
                    targetReport.aspectScore > SCORE_MIN &&
                    targetReport.areaScore > SCORE_MIN;

            if (hasImage)
            {
                highLightTarget(
                        image,
                        targetReport.boundingRectLeft,
                        targetReport.boundingRectTop,
                        targetReport.boundingRectRight,
                        targetReport.boundingRectBottom);
            }

            if (debugVision)
            {
                dashboard.displayPrintf(
                        1, "imageWidth = %d, imageHeight = %d",
                        targetReport.imageWidth,
                        targetReport.imageHeight);
                dashboard.displayPrintf(
                        2, "rectLeft = %f, rectRight = %f",
                        targetReport.boundingRectLeft,
                        targetReport.boundingRectRight);
                dashboard.displayPrintf(
                        3, "rectTop = %f, rectBottom = %f",
                        targetReport.boundingRectTop,
                        targetReport.boundingRectBottom);
                dashboard.displayPrintf(
                        4, "areaScore = %f, aspectScore = %f",
                        targetReport.areaScore,
                        targetReport.aspectScore);
                dashboard.displayPrintf(
                        5, "distance = %f, isTote = %s, deltaX/Y = %4.1f/%4.1f",
                        targetReport.distance,
                        Boolean.toString(targetReport.isTote),
                        (targetReport.boundingRectLeft +
                         targetReport.boundingRectRight -
                         targetReport.imageWidth)/2,
                        (targetReport.boundingRectTop +
                         targetReport.boundingRectBottom -
                         targetReport.imageHeight)/2);
            }

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, targetReport.toString());
            }
        }

        return hasReport? targetReport: null;
    }   //getTargetReport

    private void highLightTarget(
            Image image,
            double left, double top,
            double right, double bottom)
    {
        Rect rect =
                new Rect((int)top, (int)left,
                         (int)(bottom - top), (int)(right - left));
        NIVision.imaqDrawShapeOnImage(
                image,
                image,
                rect,
                DrawMode.DRAW_VALUE,
                ShapeMode.SHAPE_RECT,
                0.0f);
    }   //highLightTarget

    /**
     * Converts a ratio with ideal value of 1 to a score. The resulting
     * function is piecewise linear going from (0,0) to (1,100) to (2,0)
     * and is 0 for all inputs outside the range 0-2
     */
    private double ratioToScore(double ratio)
    {
        return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
    }   //ratioToScore

    private double AreaScore(ParticleReport report)
    {
        double boundingArea =
                (report.boundingRectBottom - report.boundingRectTop)*
                (report.boundingRectRight - report.boundingRectLeft);
        //Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers
        //24" of the rect.
        return ratioToScore((49/24)*report.area/boundingArea);
    }   //AreaScore

    /**
     * Method to score if the aspect ratio of the particle appears to match
     * the retro-reflective target. Target is 7"x7" so aspect should be 1
     */
    private double AspectScore(ParticleReport report)
    {
        return ratioToScore(
                ((report.boundingRectRight-report.boundingRectLeft)/
                 (report.boundingRectBottom-report.boundingRectTop)));
    }   //AspectScore

    /**
     * Computes the estimated distance to a target using the width of the
     * particle in the image. For more information and graphics showing the
     * math behind this approach see the Vision Processing section of the
     * ScreenStepsLive documentation.
     *
     * @param report The Particle Analysis Report for the particle
     * @param isLong Boolean indicating if the target is believed to be the
     *        long side of a tote
     * @return The estimated distance to the target in feet.
     */
    private double computeDistance (double rectWidth, double imageWidth)
    {
        double normalizedWidth = 2*(rectWidth)/imageWidth;
        double targetWidth = 15.1319;

        return targetWidth/
               (normalizedWidth*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
    }   //computeDistance

    private void setStreamingEnabled(boolean enabled)
    {
        final String funcName = "setStreamingEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.FUNC,
                    "enabled=%s", Boolean.toString(enabled));
        }

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (enabled)
        {
            taskMgr.registerTask(
                    moduleName,
                    this,
                    TrcTaskMgr.TaskType.POSTPERIODIC_TASK);
        }
        else
        {
            taskMgr.unregisterTask(
                    this,
                    TrcTaskMgr.TaskType.POSTPERIODIC_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setStreamingEnabled

    //
    // Implements TrcTaskMgr.Task
    //
    
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    public void stopTask(TrcRobot.RunMode runMode)
    {
    }   //stopTask

    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
        if (hasImage)
        {
            CameraServer.getInstance().setImage(image);
        }
    }   //postPeriodicTask

    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //preContinuousTask

    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class VisionTarget
