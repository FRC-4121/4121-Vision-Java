package frc.vision.process;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import frc.vision.camera.CameraConfig;
import frc.vision.load.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.ToDoubleFunction;
import java.util.stream.Collectors;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class AprilTagProcessor extends ObjectVisionProcessor<AprilTagProcessor.State> {
    protected static class State {
        Pose3d pose;
        double confidence;
    }

    private static final Transform3d coordinateConversion = new Transform3d(0, 0, 0, new Rotation3d(-Math.PI / 2, -Math.PI / 2, 0));

    protected AprilTagDetector detector;
    protected Scalar tagColor;
    protected AprilTagFieldLayout field;
    protected Confidence confidence = Confidence.distRecip();

    @FunctionalInterface
    public interface Confidence {
        double weight(Transform3d transform);
        default void reset() {}

        static Confidence distRecip() {
            return transform -> {
                double len = transform.getTranslation().getNorm();
                return len == 0 ? 0 : 1 / len;
            };
        }
    }

    public class AprilTag extends VisionObject {
        public AprilTagDetection found;
        public Transform3d pose;

        public AprilTag(AprilTagDetection found, Transform3d pose) {
            super(rectFromTag(found));
            this.found = found;
            this.pose = pose;
        }

        private static Rect rectFromTag(AprilTagDetection found) {
            double minX = found.getCornerX(0);
            double minY = found.getCornerY(0);
            double maxX = minX;
            double maxY = minY;

            for (int i = 1; i < 4; ++i) {
                double x = found.getCornerX(i);
                double y = found.getCornerY(i);

                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }

            return new Rect(new Point(minX, minY), new Point(maxX, maxY));
        }

        @Override
        public void calcAngles(CameraConfig cfg) {
            if (pose == null) super.calcAngles(cfg);
            else {
                Translation3d trans = pose.getTranslation();
                double x = trans.getX();
                double y = trans.getY();
                double z = trans.getZ();
                azimuth = Math.atan2(x, z);
                elevation = -Math.atan2(y, z);
                distance = Math.sqrt(x * x + y * y + z * z);
                offset = x / z * distance;
                rotation = pose.getRotation().getY();
                hasAngles = true;
            }
        }

        public int getId() {
            return found.getId();
        }

        public double getX() {
            return found.getCenterX();
        }

        public double getY() {
            return found.getCenterY();
        }
    }

    public static final ConcurrentHashMap<String, Collection<AprilTag>> seen = new ConcurrentHashMap<>();

    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor, Scalar tagColor, AprilTagDetector detector, AprilTagFieldLayout field) {
        super(name, cfg, rectColor);
        this.tagColor = tagColor;
        this.detector = detector;
        this.field = field;
        this.calcAngles = true;
    }
    
    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor, Scalar tagColor, AprilTagFieldLayout field) {
        super(name, cfg, rectColor);
        this.tagColor = tagColor;
        this.detector = new AprilTagDetector();
        this.field = field;
        this.calcAngles = true;
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor, Scalar tagColor, String family, AprilTagFieldLayout field) {
        this(name, cfg, rectColor, tagColor, field);
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg, AprilTagFieldLayout field) {
        this(name, cfg, new Scalar(0, 0, 255), new Scalar(255, 255, 0), field);
    }

    public AprilTagDetector getDetector() {
        return detector;
    }

    public static AprilTagDetector defaultDetector() {
        AprilTagDetector detect = new AprilTagDetector();
        AprilTagDetector.Config cfg = new AprilTagDetector.Config();
        detect.setConfig(cfg);
        detect.addFamily("tag36h11");
        return detect;
    }

    @Override
    protected Collection<VisionObject> processObjects(Mat img, CameraBase cam, Map<String, VisionProcessor> _deps) {
        return null;
    }

    @Override
    protected Collection<VisionObject> processObjects(Mat img, CameraBase cam, Map<String, VisionProcessor> _deps, Ref<State> state) {
        AprilTagDetection[] tags = new AprilTagDetection[0];
        Mat grayFrame = new Mat();
        switch (img.channels()) {
            case 1:
                img.copyTo(grayFrame);
                break;
            case 3:
                Imgproc.cvtColor(img, grayFrame, Imgproc.COLOR_BGR2GRAY);
                break;
            default:
                throw new RuntimeException("Wrong number of image channels!");
        }
        synchronized(detector) {
            tags = detector.detect(grayFrame);
        }
        // unlike the detector, the estimator seems to just be a wrapper around the config
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(cam.getConfig().poseConfig());
        Collection<AprilTag> tagCollection = Arrays.stream(tags)
            .map(obj -> new AprilTag(obj, cam.getConfig().transform.inverse().plus(estimator.estimate(obj))))
            .collect(Collectors.toList());
        seen.put(cam.getName(), tagCollection);
        if (field != null) {
            Quaternion first = null;
            Quaternion avgRot = null;
            Translation3d avgTrans = null;
            double totalWeight = 0;
            for (var tag : tagCollection) {
                if (tag.pose == null) continue;
                var basePose = field.getPose(tag.getId());
                if (basePose == null) {
                    System.out.println(1);
                    continue;
                }
                double w = confidence.weight(tag.pose);
                if (w == 0) {
                    System.out.println(2);
                    continue;
                }
                var pose = basePose.transformBy(tag.pose.inverse().plus(coordinateConversion));
                System.out.println(pose.getTranslation());
                var vec = pose.getTranslation().times(w);
                var quat = pose.getRotation().getQuaternion().times(w);
                if (first == null) {
                    first = quat;
                    avgRot = quat;
                    avgTrans = vec.times(w);
                } else {
                    if (first.dot(quat) < 0) quat = quat.times(-1);
                    avgRot = avgRot.plus(quat);
                    avgTrans = avgTrans.plus(vec.times(w));
                }
                totalWeight += w;
            }
            state.inner = new State();
            state.inner.confidence = totalWeight;
            if (totalWeight != 0) state.inner.pose = new Pose3d(avgTrans.div(totalWeight), new Rotation3d(avgRot.divide(totalWeight)));
        }
        return tagCollection.stream().collect(Collectors.toList());
    }

    @Override
    public void toNetworkTableStateful(NetworkTable table, Ref<ObjectVisionProcessor<State>.Storage> state) {
        super.toNetworkTableStateful(table, state);
        NetworkTable table_ = table.getSubTable(name);
        var value = table_.getValue("filter");
        long[] filter = new long[0];
        switch (value.getType()) {
            case kIntegerArray: // we really just want an integer array
                filter = value.getIntegerArray();
                break;
            case kDoubleArray: // but for some reason NT coerces it to double[]
                double[] vals = value.getDoubleArray();
                filter = new long[vals.length];
                for (int i = 0; i < vals.length; ++i) filter[i] = (long)vals[i];
                break;
        }
        int size = state.inner.seen.size();
        long[] ids = new long[size];
        int i = 0;
        AprilTag best = null;
        for (VisionObject obj : state.inner.seen) {
            AprilTag tag = (AprilTag)obj;
            ids[i] = tag.getId();
            if (Arrays.stream(filter).anyMatch(f -> f == tag.getId())) {
                if (best == null || best.distance > tag.distance) best = tag;
            }
            ++i;
        }
        table_.putValue("ids", NetworkTableValue.makeIntegerArray(ids));
        if (state.inner.additional != null) {
            var pose = state.inner.additional.pose;
            table_.putValue("pose/confidence", NetworkTableValue.makeDouble(state.inner.additional.confidence));
            if (pose != null) {
                var rot = pose.getRotation();
                table_.putValue("pose/x", NetworkTableValue.makeDouble(pose.getX()));
                table_.putValue("pose/y", NetworkTableValue.makeDouble(pose.getY()));
                table_.putValue("pose/z", NetworkTableValue.makeDouble(pose.getZ()));
                table_.putValue("pose/roll",  NetworkTableValue.makeDouble(rot.getX()));
                table_.putValue("pose/pitch", NetworkTableValue.makeDouble(rot.getY()));
                table_.putValue("pose/yaw",   NetworkTableValue.makeDouble(rot.getZ()));
            }
        }
        if (best != null) {
            table_.putValue("best/found", NetworkTableValue.makeBoolean(true));
            {
                var entry = table_.getEntry("best/id");
                if (entry.getType() == NetworkTableType.kDouble) entry.setDouble(best.getId());
                else entry.setInteger(best.getId());
            }
            table_.putValue("best/d", NetworkTableValue.makeDouble(best.distance));
            table_.putValue("best/a", NetworkTableValue.makeDouble(best.azimuth));
            table_.putValue("best/e", NetworkTableValue.makeDouble(best.elevation));
            table_.putValue("best/o", NetworkTableValue.makeDouble(best.offset));
            table_.putValue("best/r", NetworkTableValue.makeDouble(best.rotation));
        } else {
            table_.putValue("best/found", NetworkTableValue.makeBoolean(false));
        }
    }

    @Override
    public void drawOnImageStateful(Mat img, Ref<ObjectVisionProcessor<State>.Storage> state) {
        super.drawOnImageStateful(img, state);
        for (VisionObject obj : state.inner.seen) {
            AprilTagDetection tag = ((AprilTag)obj).found;
            Imgproc.polylines(
                img,
                Arrays.asList(new MatOfPoint(
                    new Point(tag.getCornerX(0), tag.getCornerY(0)),
                    new Point(tag.getCornerX(1), tag.getCornerY(1)),
                    new Point(tag.getCornerX(2), tag.getCornerY(2)),
                    new Point(tag.getCornerX(3), tag.getCornerY(3))
                )),
                true,
                tagColor,
                5,
                -1
            );
            double h = ((AprilTag)obj).height;
            h *= 0.1;
            Imgproc.putText(
                img,
                String.valueOf(tag.getId()),
                new Point(tag.getCenterX() - h, tag.getCenterY()),
                Imgproc.FONT_HERSHEY_PLAIN,
                h * 0.25,
                tagColor,
                2
            );
        }
    }

    public static class Config extends ProcessorConfig {
        public ArrayList<String> family;
        public String fieldLayout;
    }
    public static class Factory extends ProcessorFactory {
        @Override
        public String typeName() {
            return "apriltag";
        }
        @Override
        public Class<Config> configType() {
            return Config.class;
        }
        @Override
        public AprilTagProcessor create(String name, ProcessorConfig cfg) {
            Config cfg_ = (Config)cfg;
            AprilTagProcessor out = new AprilTagProcessor(name, cfg, cfg_.fieldLayout == null ? null : AprilTagFieldLayout.get(cfg_.fieldLayout));
            if (cfg_.family != null) {
                for (String family : cfg_.family) {
                    out.getDetector().addFamily(family);
                }
            }
            return out;
        }
    }
}
