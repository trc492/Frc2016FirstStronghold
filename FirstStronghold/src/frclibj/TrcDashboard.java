package frclibj;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;

public class TrcDashboard extends SmartDashboard
{
    public static final int maxTextLines = 8;
    private static final String textLinePrefix = "TextLine";

    public static void textPrintf(int lineNum, String format, Object... args)
    {
        if (lineNum >= 0 && lineNum < maxTextLines)
        {
            SmartDashboard.putString(
                    textLinePrefix + lineNum,
                    String.format(format, args));
        }
    }   //textPrintf

    public static void clearTextLines()
    {
        for (int i = 0; i < maxTextLines; i++)
        {
            SmartDashboard.putString(textLinePrefix + i, "");
        }
    }   //clearTextLines

    public static boolean getBoolean(String key, boolean defaultValue)
    {
        boolean value;

        try
        {
            value = getBoolean(key);
        }
        catch (TableKeyNotDefinedException e)
        {
            putBoolean(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getBoolean

    public static double getNumber(String key, double defaultValue)
    {
        double value;

        try
        {
            value = getNumber(key);
        }
        catch (TableKeyNotDefinedException e)
        {
            putNumber(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getNumber

    public static String getString(String key, String defaultValue)
    {
        String value;

        try
        {
            value = getString(key);
        }
        catch (TableKeyNotDefinedException e)
        {
            putString(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getString

}   //class TrcDashboard
