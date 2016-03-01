package frc492;

import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.Relay;
import frclib.FrcVision;
import frclib.FrcVision.ParticleReport;
import trclib.TrcDbgTrace;
import hallib.HalDashboard;

public class VisionTarget
{
    private static final String moduleName = "VisionTarget";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean debugVision = false;
    private HalDashboard dashboard = HalDashboard.getInstance();

    public class TargetReport
    {
        int imageWidth;
        int imageHeight;
        NIVision.Rect rect;
        double areaScore;
        double aspectScore;
        double distance;
        boolean isTote;

        public String toString()
        {
            return  "\nimageWidth  = " + imageWidth +
                    "\nimageHeight = " + imageHeight +
                    "\nrectLeft    = " + rect.left +
                    "\nrectTop     = " + rect.top +
                    "\nrectRight   = " + rect.left + rect.width +
                    "\nrectBottom  = " + rect.top + rect.height +
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
    private Image image;
    private Range[] colorThresholds;
    private ParticleFilterCriteria2[] filterCriteria;
    private ParticleFilterOptions2 filterOptions;
    private TargetReport targetReport;
    private FrcVision visionTask;

    public VisionTarget(FrcVision.ImageProvider imageProvider)
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
        visionTask = new FrcVision(
                imageProvider,
                ImageType.IMAGE_RGB,
                ColorMode.HSV,
                colorThresholds,
                false,
                filterCriteria,
                filterOptions);
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
                targetReport.rect.left = (int)
                        reports.elementAt(bestTarget1Index).boundingRectLeft;
                targetReport.rect.width = (int)
                        (reports.elementAt(bestTarget1Index).boundingRectRight -
                         reports.elementAt(bestTarget1Index).boundingRectLeft);
                targetReport.rect.top = (int)
                        reports.elementAt(bestTarget1Index).boundingRectTop;
                targetReport.rect.height = (int)
                        (reports.elementAt(bestTarget1Index).boundingRectBottom -
                         reports.elementAt(bestTarget1Index).boundingRectTop);
                targetReport.aspectScore = AspectScore(reports.elementAt(bestTarget1Index));
                targetReport.areaScore = AreaScore(reports.elementAt(bestTarget1Index));
            }
            else
            {
                targetReport.rect.left = (int)
                    Math.min(reports.elementAt(bestTarget1Index).boundingRectLeft,
                             reports.elementAt(bestTarget2Index).boundingRectLeft);
                targetReport.rect.width = (int)
                    Math.max(reports.elementAt(bestTarget1Index).boundingRectRight -
                             reports.elementAt(bestTarget1Index).boundingRectLeft,
                             reports.elementAt(bestTarget2Index).boundingRectRight -
                             reports.elementAt(bestTarget2Index).boundingRectLeft);
                targetReport.rect.top = (int)
                    Math.min(reports.elementAt(bestTarget1Index).boundingRectTop,
                             reports.elementAt(bestTarget2Index).boundingRectTop);
                targetReport.rect.height = (int)
                    Math.max(reports.elementAt(bestTarget1Index).boundingRectBottom -
                             reports.elementAt(bestTarget1Index).boundingRectTop,
                             reports.elementAt(bestTarget2Index).boundingRectBottom -
                             reports.elementAt(bestTarget2Index).boundingRectTop);
                targetReport.aspectScore =
                        (AspectScore(reports.elementAt(bestTarget1Index)) +
                         AspectScore(reports.elementAt(bestTarget2Index)))/2;
                targetReport.areaScore =
                        (AreaScore(reports.elementAt(bestTarget1Index)) +
                         AreaScore(reports.elementAt(bestTarget2Index)))/2;
            }
            targetReport.distance =
                    computeDistance(targetReport.rect.width, targetReport.imageWidth);
            targetReport.isTote =
                    targetReport.aspectScore > SCORE_MIN &&
                    targetReport.areaScore > SCORE_MIN;

            if (debugVision)
            {
                dashboard.displayPrintf(
                        1, "imageWidth = %d, imageHeight = %d",
                        targetReport.imageWidth,
                        targetReport.imageHeight);
                dashboard.displayPrintf(
                        2, "rectLeft = %f, rectRight = %f",
                        targetReport.rect.left,
                        targetReport.rect.left + targetReport.rect.width);
                dashboard.displayPrintf(
                        3, "rectTop = %f, rectBottom = %f",
                        targetReport.rect.top,
                        targetReport.rect.top + targetReport.rect.height);
                dashboard.displayPrintf(
                        4, "areaScore = %f, aspectScore = %f",
                        targetReport.areaScore,
                        targetReport.aspectScore);
                dashboard.displayPrintf(
                        5, "distance = %f, isTote = %s, deltaX/Y = %4.1f/%4.1f",
                        targetReport.distance,
                        Boolean.toString(targetReport.isTote),
                        (targetReport.rect.left + targetReport.rect.width/2.0 -
                         targetReport.imageWidth)/2.0,
                        (targetReport.rect.top + targetReport.rect.height/2.0 -
                         targetReport.imageHeight)/2.0);
            }

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, targetReport.toString());
            }
        }

        return hasReport? targetReport: null;
    }   //getTargetReport

    public NIVision.Rect getLastTargetRect()
    {
        return null;
    }   //getLastTargetRect

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

}   //class VisionTarget
