package frc.robot.utils;

import java.util.Map.Entry;
import java.util.TreeMap;

public class ShotCalculator {
    
    // TODO: replace with real values
    public static final TreeMap<Double, ShooterParams> m_shooterMap = new TreeMap<>();
    static {
        m_shooterMap.put(1.0, new ShooterParams(1000, 0));
        m_shooterMap.put(2.0, new ShooterParams(2000, 0));
        m_shooterMap.put(3.0, new ShooterParams(3000, 0));
        m_shooterMap.put(4.0, new ShooterParams(4000, 0));
        m_shooterMap.put(70.0, new ShooterParams(1900, 0));
        m_shooterMap.put(80.0, new ShooterParams(2000, 0));
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

        return new ShooterParams(rpm, tof);
    }

    /**
     * Actual linear interpolation method.
     * @param start
     * @param end
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
