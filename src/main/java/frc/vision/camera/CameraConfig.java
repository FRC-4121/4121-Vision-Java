package frc.vision.camera;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import frc.vision.load.Typed;
import java.util.ArrayList;
import org.opencv.core.*;

// Bean for camera config
public class CameraConfig extends Typed {
    public int width = -1;
    public int height = -1;
    public boolean enforceSize = false;
    public float fov = -1;
    public ArrayList<String> vlibs;
    public int cropBottom;
    public int fpsThrottle = Integer.MAX_VALUE;
    public int lockTimeout = Integer.MAX_VALUE;
    public StreamConfig stream;
    public int crosshair = 0;
    public String bottomLeft;
    public String bottomRight;
    public float offsetX;
    public float offsetY;
    public float offsetZ;

    public void updateFrom(CameraConfig other) {
        if (width < 0) width = other.width;
        if (height < 0) height = other.height;
        if (fov < 0) fov = other.fov;
        if (offsetX < 0)
        if (other.vlibs != null) {
            if (vlibs == null) vlibs = other.vlibs;
            else vlibs.addAll(other.vlibs);
        }
        if (other.fpsThrottle < fpsThrottle) fpsThrottle = other.fpsThrottle;
        if (crosshair == 0) crosshair = other.crosshair;
        if (bottomLeft == null) bottomLeft = other.bottomLeft;
        if (bottomRight == null) bottomRight = other.bottomRight;
    }
    public Mat camMat() {
        float f = 0.5f / (float)Math.tan(fov * Math.PI / 360) * width;
        return new MatOfFloat(
            f, 0, width / 2,
            0, f, height / 2,
            0, 0, 1
        ).reshape(1, 3); //Matt is meany face :P
    }
    public AprilTagPoseEstimator.Config poseConfig() {
        float f = 0.5f / (float)Math.tan(fov * Math.PI / 360) * width;
        return new AprilTagPoseEstimator.Config(6.5, f, f, width / 2, height / 2);
    }
}
