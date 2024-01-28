package frc.robot.Util;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double cubic(double x, double weight) {
        return weight * x * x * x + (1.0 - weight) * x;
    }

    public static double applyScaledDeadband(double x, double db, double max) {
        if (Math.abs(x) < db) {
            return 0.0;
        }

        double sgnX = Math.copySign(1.0, x);
        return (x - sgnX * db) / (max - db);
    }

    public static double applyScaledDeadband(double x, double db) {
        return applyScaledDeadband(x, db, 1.0);
    }

    public static double applyCubicScaledDeadband(double x, double db, double weight) {
        if (Math.abs(x) < db)
            return 0.0;

        double scaledX = applyScaledDeadband(x, db);
        return cubic(scaledX, weight);
    }

    public static Translation2d apply2DWithAngleDeadband(double X, double Y, double centerDeadzoneRadius,
            double angularDeadzoneDegrees, double cubicWeight) {
        // setup helper variables
        double angleRadians = Math.atan2(Y, X);
        double magnitude = Math.sqrt(Math.pow(X, 2) + Math.pow(Y, 2));
        double scaledX = applyScaledDeadband(X, centerDeadzoneRadius);
        double scaledY = applyScaledDeadband(Y, centerDeadzoneRadius);

        // SmartDashboard.putNumber("angle", Math.toDegrees(angleRadians));
        // SmartDashboard.putNumber("mag", magnitude);
        // SmartDashboard.putNumber("scaled X", scaledX);
        // SmartDashboard.putNumber("scaled Y", scaledY);

        // center radius
        if (magnitude < centerDeadzoneRadius) {
            return new Translation2d(0, 0);
        }

        // round to nearest axis [-180, -90, 0, 90]
        double roundTo = Math.PI / 2.0;
        // divide measured angle by roundTo interval to get closest integer multiple of
        // roundTo
        // multiply by roundTo to get the actual angle of the axis
        double closestAxis = roundTo * Math.round(angleRadians / roundTo);
        // convert angle Deadband to radians
        double angleDb = Units.degreesToRadians(angularDeadzoneDegrees / 2.0);

        // scale and cube magnitude
        double cubedMagnitude = applyCubicScaledDeadband(magnitude, centerDeadzoneRadius, cubicWeight);
        // deadband should return [-1.0, 1.0] already, but safety check here
        if (Math.abs(cubedMagnitude) > 1.0) {
            cubedMagnitude = Math.copySign(1.0, cubedMagnitude);
        }

        // scale angle, find difference to closest axis
        double angleDistanceToAxis = closestAxis - angleRadians;
        // apply deadband to the angle, using a max input value of roundTo/2
        // since the deadband should scale from each axis to the mid-angle between them
        // equally
        // deadband returns a value [-1.0, 1.0]
        // multiply by half the distance in between axis,
        // since that's where the scaled values from each axis must be equal
        double scaledDifferenceRadians = (roundTo / 2.0)
                * applyScaledDeadband(angleDistanceToAxis, angleDb, roundTo / 2.0);
        // re-reference the angle to the axis it was closest to
        double scaledAngleRadians = closestAxis - scaledDifferenceRadians;

        SmartDashboard.putNumber("closestAxis", Math.toDegrees(closestAxis));
        SmartDashboard.putNumber("cubed mag", cubedMagnitude);
        SmartDashboard.putNumber("angleDistanceToAxis in degrees", Math.toDegrees(angleDistanceToAxis));
        SmartDashboard.putNumber("scaledDifferenceRadians in degrees", Math.toDegrees(scaledDifferenceRadians));
        SmartDashboard.putNumber("scaledAngleRadians in degrees", Math.toDegrees(scaledAngleRadians));

        return new Translation2d(cubedMagnitude, Rotation2d.fromRadians(scaledAngleRadians));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                System.out.println("NIS: " + nis.getDisplayName());
                if (nis != null && "eth0".equals(nis.getDisplayName())) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                        }
                        String addr = ret.toString();
                        System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);
                        return addr;
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Skipping adaptor: " + nis.getDisplayName());
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}