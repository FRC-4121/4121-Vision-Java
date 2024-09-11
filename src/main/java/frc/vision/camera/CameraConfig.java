import java.util.ArrayList;

/// POJO for camera config
class CameraConfig {
    public int width;
    public int height;
    public int fps;
    public float fov;
    public ArrayList<String> vlibs;

    public int cropBottom;
    public Integer port;
    public String path;

    public void updateFrom(CameraConfig other) {
        if (width == 0) width = other.width;
        if (height == 0) height = other.height;
        if (fps == 0) fps = other.fps;
        if (fov == 0) fov = other.fov;
        vlibs.addAll(other.vlibs);
    }
}
