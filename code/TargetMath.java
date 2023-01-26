import javafx.geometry.Point2D;
import javafx.geometry.Point3D;

/**
 * Collection of methods for doing math regarding aiming at targets from PhotonVision outputs
 *
 * @author Benjamin Rueter
 */
public class TargetMath {

    /**
     * 
     * @param angle raw angle output from target data in radians
     * @return usable angle in degrees from -90 to 90
     */
    public static double convert(double angle) {
        angle = Math.toDegrees(angle);
        if(angle > 0) {
            return 180 - angle;
        }
        else {
            return -(180 + angle);
        }
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @return angle from robot to target in given axis in degrees
     */
    public static double angleHorizontal(PhotonTrackedTarget target) {
        double distance = getDistance(target);
        double offset = getHorizontal(target);
        return Math.toDegrees(Math.atan(offset/distance));
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return angle from robot to target in given axis in degrees
     */
    public static double angleVertical(PhotonTrackedTarget target) {
        double distance = getDistance(target);
        double offset = getVertical(target);
        return Math.toDegrees(Math.atan(offset/distance));
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @return considering the field to be a 2d grid, the x and y offset from a given target
     */
    public static Point2D gridOffset(PhotonTrackedTarget target) {
        double distance = getDistance(target);
        double angle = getYaw(target);
        
        double x, y;
        angle = convert(angle);

        x = Math.sin(angle) * distance;
        y = Math.cos(angle) * distance;
        return new Point2D(x, y);
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @return distance from target in meters
     */
    public static double getX(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getX();
    }
    //alias
    public static double getDistance(PhotonTrackedTarget target) {
        return getX(target);
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @return horizontal offset from target in meters
     */
    public static double getY(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getY();
    }
    //alias
    public static double getHorizontal(PhotonTrackedTarget target) {
        return getY(target);
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return vertical offset from target in meters
     */
    public static double getZ(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getZ();
    }
    //alias
    public static double getVertical(PhotonTrackedTarget target) {
        return getZ(target);
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return target roll in radians
     */
    public static double getRoll(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getRotation().getRoll();
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return target pitch in radians
     */
    public static double getPitch(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getRotation().getPitch();
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return target yaw in radians
     */
    public static double getYaw(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getRotation().getYaw();
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return target roll in degrees
     */
    public static double getRollDegrees(PhotonTrackedTarget target) {
        return Math.toDegrees(target.getBestCameraToTarget().getRotation().getRoll());
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return target pitch in degrees
     */
    public static double getPitchDegrees(PhotonTrackedTarget target) {
        return Math.toDegrees(target.getBestCameraToTarget().getRotation().getPitch());
    }
    
    /**
     * 
     * @param target apriltag to grab data from
     * @return target yaw in degrees
     */
    public static double getYawDegrees(PhotonTrackedTarget target) {
        return Math.toDegrees(target.getBestCameraToTarget().getRotation().getYaw());
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @return x y and z offset
     */
    public static Point3D transform(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTransform();
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @return x y and z rotation (roll, pitch, yaw)
     */
    public static Point3D rotation(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getRotation();
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @return target ID
     */
    public static int getID(PhotonTrackedTarget target) {
        return target.getFiducialId() ;
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @param headOnDistance distance to shoot the ball from if we are lined up perfectly
     * @param offset distance from the apriltag to the point we want the ball to land
     * @return distance to PID to
     */
    public static double retargetDistance(PhotonTrackedTarget target, double headOnDistance, double offset) {
        double y1 = gridOffset(target).y;
        double y2 = offset;
        double y3 = y1 + y2;
        double x = gridOffset(target).x;
        
        double angle1 = Math.toDegrees(Math.atan(y1/x));
        double angle2 = Math.atan(y3/x);
        return angle2 - angle1;
    }

    /**
     * 
     * @param target apriltag to grab data from
     * @param headOnDistance distance to shoot the ball from if we are lined up perfectly
     * @param offset distance from the apriltag to the point we want the ball to land
     * @return andle to PID to
     */
    public static double retargetAngle(PhotonTrackedTarget target, double headOnDistance, double offset) {
        double y1 = gridOffset(target).y;
        double y2 = offset;
        double y3 = y1 + y2;
        double x = gridOffset(target).x;
        
        double angle1 = Math.toDegrees(Math.atan(y1/x));
        double angle2 = Math.atan(y3/x);
        double bestDistance = angle2 - angle1;

        double z1 = headOnDistance;
        double z2 = Math.sin(angle2)*y3;

        return z2-z1;
    }
}