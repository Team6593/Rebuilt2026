package frc.robot.utils;

import java.util.Map.Entry;
import java.util.TreeMap;

public class ShotCalculator {
    
    // TODO: replace with real values
    public static final TreeMap<Double, ShooterParams> m_shooterMap = new TreeMap<>();
    static {
        m_shooterMap.put(55.0, new ShooterParams(1875, 0));
        m_shooterMap.put(60.0, new ShooterParams(1880, 0));
        m_shooterMap.put(65.0, new ShooterParams(1900, 0));
        m_shooterMap.put(70.0, new ShooterParams(1910, 0));
        m_shooterMap.put(75.0, new ShooterParams(1920, 0));
        m_shooterMap.put(80.5, new ShooterParams(1930, 0));
        m_shooterMap.put(85.0, new ShooterParams(1940, 0));
        m_shooterMap.put(90.0, new ShooterParams(1950, 0));
        m_shooterMap.put(95.0, new ShooterParams(1980, 0));
        m_shooterMap.put(100.0, new ShooterParams(2045, 0));
        m_shooterMap.put(105.0, new ShooterParams(2060, 0));
        m_shooterMap.put(110.0, new ShooterParams(2125, 0));
        m_shooterMap.put(115.0, new ShooterParams(2150, 0));
        m_shooterMap.put(119.0, new ShooterParams(2175, 0));
        m_shooterMap.put(125.0, new ShooterParams(2225, 0));
        m_shooterMap.put(130.0, new ShooterParams(2250, 0));
    }

    /**
     * Method that uses linear interpolation to calculate RPM from distance.
     * @param distance - Current distance from target.
     * @return New {@link ShooterParams} object.
     */
    public static ShooterParams lerpGet(double distance) {
        if (m_shooterMap.containsKey(distance)) {
            // Exact key found, return directly
            return m_shooterMap.get(distance);
        }

        Double lowerKey = m_shooterMap.floorKey(distance);
        Double upperKey = m_shooterMap.ceilingKey(distance);

        if (lowerKey == null) {
            // key is below the smallest key
            return m_shooterMap.get(upperKey);
        }

        if (upperKey == null) {
            // key is above the largest key
            return m_shooterMap.get(lowerKey);
        }

        ShooterParams lowerParams = m_shooterMap.get(lowerKey);
        ShooterParams upperParams = m_shooterMap.get(upperKey);

        double ratio = (distance - lowerKey) / (upperKey - lowerKey);

        // Linear interpolation for rpm and tof
        double rpm = interpolate(lowerParams.rpm, upperParams.rpm, ratio);
        double tof = interpolate(lowerParams.tof, upperParams.tof, ratio);

        if (distance > 90) {
            rpm += 50;
        }

        return new ShooterParams(rpm, tof);
    }

    /**
     * Actual linear interpolation method.
     * @param start
     * @param endbor
     * @param ratio
     * @return double
     */
    private static double interpolate(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }

    /**
     * Gets the horizontal velocity needed at the current distance.
     * @param distance
     * @return rpm
     */
    public static double getHorizontalVelocity(double distance) {
        ShooterParams params = m_shooterMap.get(distance);
        return distance / params.tof;
    }

    /**
     * Method that goes from velocity to the distance by doing a binary search
     * through the tree map.
     * @param velocity - RPM
     * @return distance
     */
    public static double velocityToEffectiveDistance(double velocity) {
        for (Entry<Double, ShooterParams> entry : m_shooterMap.entrySet()) {
        double dist = entry.getKey();
        double vel = dist / entry.getValue().tof;
        if (vel >= velocity) {
            return dist;
        }
        } return m_shooterMap.lastKey();
    }

    public static double calculatedAdjustedRpm(double requiredVelocity) {
        double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
        return m_shooterMap.get(effectiveDistance).rpm;
    }

}
