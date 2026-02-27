package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Fuel {

    private Translation3d position;
    private Translation3d velocity; // feet per second
    private static final double gravity = Units.feetToMeters(-32.2);

    /**
     * Class for the fuel game piece. This class is used for simulation and visualization purposes.
     * @param launchPose
     * @param launchVelocityFps
     */
    public Fuel(Pose3d launchPose, double launchVelocityFps) {
        this.position = launchPose.getTranslation();

        // Convert launch angle to a velocity vector
        this.velocity = new Translation3d(
            Units.feetToMeters(launchVelocityFps), 0, 0)
            .rotateBy(new Rotation3d(0, launchPose.getRotation().getY(), launchPose.getRotation().getZ()));       
    }

    /**
     * Method that updates the current position of fuel.
     * @param dt
     */
    public void update(double dt) {
        // Position: position = initial position + velocity * time
        position = position.plus(velocity.times(dt));

        // Velocity: velocity = initial velocity + acceleration * time
        velocity = new Translation3d(velocity.getX(), velocity.getY(), velocity.getZ() + (gravity * dt));

        System.out.println(position.getZ());
    }

    /**
     * Returns the current pose of the generated fuel.
     * @return new {@link Pose3d} object
     */
    public Pose3d getPose() {
        return new Pose3d(position, new Rotation3d());
    }

    /**
     * Returns whether or not the fuel hit the floor (RIP).
     * @return boolean
     */
    public boolean isExpired() {
        return position.getZ() < 0;
    }

}
