package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import frc.vision.load.ProcessorFactory;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.stream.IntStream;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class Coral2025Processor extends InstancedVisionProcessor<Coral2025Processor.State> {
    private static class TaggedRect extends RotatedRect {
        int zone = Integer.MAX_VALUE;
        public TaggedRect(RotatedRect rect) {
            super(rect.center, rect.size, rect.angle);
        }
        public TaggedRect(RotatedRect rect, int zone) {
            this(rect);
            this.zone = zone;
        }
        public boolean isSimilar(RotatedRect other, double dmax2, double amax, double rmax) {
            double da1 = Math.abs(angle - other.angle);
            double da2 = 2 * Math.PI - da1;
            if (Math.min(da1, da2) > rmax) return false;

            double dx = center.x - other.center.x;
            double dy = center.y - other.center.y;
            if (dx * dx + dy * dy > dmax2) return false;

            double ar = size.width * other.size.height / (size.height * other.size.width);
            return ar < amax && ar > 1 / amax;
        }
    }
    protected static class State {
        Rect overallCrop;
        List<TaggedRect> zones;
        List<TaggedRect> detections;
    }

    public static class Position {
        // expected distance to the right
        public double x;
        // expected distance upwards
        public double y;
        // expected distance backwards
        public double z;
        // expected angle, rotated toward the camera
        public double angle;
    }
    public static class Config extends RectVisionProcessor.Config {
        // initial width, in april tag widths
        public double iw = Double.POSITIVE_INFINITY;
        // initial height, in april tag heights
        public double ih = Double.POSITIVE_INFINITY;
        // initial distance to the right, in april tag widths
        public double idx = 0;
        // initial distance upwards, in april tag heights
        public double idy = 0;
        // initial distance between the coral plane and the tag plane, in april tag widths
        public double idz = 0;
        // maximum square distance between the measured and expected center, in pixels
        public double dmax2 = Double.POSITIVE_INFINITY;
        // maximum aspect ratio ratio (not a typo)
        public double amax = Double.POSITIVE_INFINITY;
        // maximum rotation difference
        public double rmax = Math.PI;

        // camera to get april tags from
        public String tagCam;
        // allowed april tags for us to search for
        public ArrayList<Integer> recognizedTags = new ArrayList<>();
        // available positions to search for
        public ArrayList<Position> positions = new ArrayList<>();
    }
    public static class Factory extends ProcessorFactory {
        @Override
        public String typeName() {
            return "coral";
        }
        @Override
        public Class<Config> configType() {
            return Config.class;
        }
        @Override
        public Coral2025Processor create(String name, ProcessorConfig cfg) {
            return new Coral2025Processor(name, (Config)cfg);
        }
    }

    public Coral2025Processor(String name, Config cfg) {
        super(name, cfg);
    }

    @Override
    public Config getConfig() {
        return (Config)super.getConfig();
    }

    @Override
    protected void processStateful(Mat img, CameraBase cam, Map<String, VisionProcessor> deps, Ref state) {
        Config cfg = getConfig();

        state.inner = new State();

        if (cfg.tagCam == null) return;
        var seen = AprilTagProcessor.seen.get(cfg.tagCam);
        if (seen == null) return;

        var tags = seen.stream()
            .filter(a -> cfg.recognizedTags.contains(a.getId()))
            .toArray(len -> new AprilTagProcessor.AprilTag[len]);

        Rect crop = Arrays.stream(tags)
            .map(a -> {
                double w = cfg.iw * a.width;
                double h = cfg.ih * a.height;
                double x = (cfg.idx + cfg.idz * Math.sin(a.approachAngle)) * a.width;
                double y = cfg.idy * a.height;
                return new Rect((int)x, (int)y, (int)w, (int)h);
            })
            .reduce((a, b) -> mergeRects(a, b))
            .orElse(null);

        if (crop == null) return;

        TaggedRect[] rects = Arrays.stream(tags)
            .flatMap(a -> {
                double sa = Math.sin(a.approachAngle);
                return IntStream.range(0, cfg.positions.size()).mapToObj(i -> {
                    Position p = cfg.positions.get(i);
                    double angle = Math.atan(Math.tan(p.angle) / sa);
                    double x = (p.x + p.z * sa) * a.width;
                    double y = p.y * a.height;
                    return new TaggedRect(
                        new RotatedRect(new Point(x, y), new Size(cfg.width, cfg.height), angle),
                        a.getId() * cfg.positions.size() + i
                    );
                });
            })
            .toArray(len -> new TaggedRect[len]);

        state.inner.overallCrop = crop;
        state.inner.zones = Arrays.asList(rects);
        state.inner.detections = new ArrayList<>();

        Mat cropped = crop == null ? img : new Mat(img, crop);

        Mat m1 = new Mat(); // blurred, reused for filtered
        Imgproc.blur(cropped, m1, new Size(3, 3));

        Mat m2 = new Mat(); // HSV, reused for blurred filter
        Imgproc.cvtColor(m1, m2, Imgproc.COLOR_BGR2HSV);
        Core.inRange(
            m2,
            new Scalar(cfg.hmin, cfg.smin, cfg.vmin),
            new Scalar(cfg.hmax, cfg.smax, cfg.vmax),
            m1
        );
        Imgproc.GaussianBlur(m1, m2, new Size(5, 5), 0);
        
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(m2, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.sort(Comparator.comparing(c -> Integer.MAX_VALUE - Imgproc.contourArea(c)));

        for (MatOfPoint c : contours) {
            if (Imgproc.contourArea(c) < cfg.minArea) break;
            TaggedRect rect = new TaggedRect(Imgproc.minAreaRect(new MatOfPoint2f(c.toArray()))); // this copying is dumb, but it's the only way

            for (TaggedRect cmp : rects) {
                if (cmp.isSimilar(rect, cfg.dmax2, cfg.rmax, cfg.amax)) {
                    rect.zone = cmp.zone;
                    break;
                }
            }

            state.inner.detections.add(rect);
        }
    }

    @Override
    protected void toNetworkTableStateful(NetworkTable table, Ref state) {
        if (state.inner == null) return;
        long[] seen = state.inner.detections
            .stream()
            .mapToLong(d -> d.zone)
            .filter(z -> z < Integer.MAX_VALUE)
            .toArray();
        table.getSubTable(name).putValue("seen", NetworkTableValue.makeIntegerArray(seen));
    }

    @Override
    protected void drawOnImageStateful(Mat img, Ref state) {
        if (state.inner.overallCrop == null) return;
        Imgproc.rectangle(
            img,
            state.inner.overallCrop.tl(),
            state.inner.overallCrop.br(),
            new Scalar(255, 0, 0),
            2
        );

        ArrayList<MatOfPoint> pts = new ArrayList<>();
        for (TaggedRect zone : state.inner.zones) {
            MatOfPoint points = new MatOfPoint();
            Imgproc.boxPoints(zone, points);
            pts.add(points);
        }
        ArrayList<MatOfPoint> good = new ArrayList<>();
        ArrayList<MatOfPoint> bad = new ArrayList<>();
        for (TaggedRect detect : state.inner.detections) {
            MatOfPoint points = new MatOfPoint();
            Imgproc.boxPoints(detect, points);
            if (detect.zone == Integer.MAX_VALUE) bad.add(points);
            else good.add(points);
        }
        Imgproc.drawContours(img, pts,  -1, new Scalar(0, 0, 0));
        Imgproc.drawContours(img, good, -1, new Scalar(0, 255, 0));
        Imgproc.drawContours(img, bad,  -1, new Scalar(0, 0, 255));
    }

    private static Rect mergeRects(Rect a, Rect b) {
        double tlx = Math.min(a.tl().x, a.tl().x);
        double tly = Math.min(a.tl().y, b.tl().y);
        double brx = Math.min(a.br().x, a.br().x);
        double bry = Math.min(a.br().y, b.br().y);

        return new Rect(new Point(tlx, tly), new Point(brx, bry));
    }
}
