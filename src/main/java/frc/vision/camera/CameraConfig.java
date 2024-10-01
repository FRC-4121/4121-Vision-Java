package frc.vision.camera;

import frc.vision.load.Typed;
import java.util.ArrayList;

// Bean for camera config
public class CameraConfig extends Typed {
    public int width;
    public int height;
    public float fov;
    public ArrayList<String> vlibs;
    public int cropBottom;

    public void updateFrom(CameraConfig other) {
        if (width == 0) width = other.width;
        if (height == 0) height = other.height;
        if (fov == 0) fov = other.fov;
        vlibs.addAll(other.vlibs);
    }
}
