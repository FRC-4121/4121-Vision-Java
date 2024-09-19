import org.opencv.core.*;

// The base vision object is just a rectangle with a name.
class VisionObject extends Rect {
    VisionObject() {}
    VisionObject(int x, int y, int w, int h) {
        super(x, y, w, h);
    }
    VisionObject(Point p1, Point p2) {
        super(p1, p2);
    }
    VisionObject(Point p, Size s) {
        super(p, s);
    }
    VisionObject(Rect r) {
        super(r.x, r.y, r.width, r.height);
    }
    public String getName() {
        return "unknown";
    }
    public double offset() {
        // TODO
        return 0;
    }
}
