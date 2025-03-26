package frc.vision.process;

import com.google.gson.*;
import edu.wpi.first.math.geometry.*;
import java.io.Reader;
import java.lang.reflect.Type;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class AprilTagFieldLayout {
    private double length;
    private double width;
    private Map<Integer, Pose3d> tags;
    public static ConcurrentHashMap<String, AprilTagFieldLayout> fields = new ConcurrentHashMap<>();

    public AprilTagFieldLayout(double length, double width, Map<Integer, Pose3d> tags) {
        this.length = length;
        this.width = width;
        this.tags = tags;
    }
    public Pose3d getPose(int id) {
        return tags.get(id);
    }
    public double fieldLength() {
        return length;
    }
    public double fieldWidth() {
        return width;
    }

    public static AprilTagFieldLayout load(String name, Reader file) throws JsonParseException, JsonIOException {
        Gson gson = new GsonBuilder().registerTypeAdapter(AprilTagFieldLayout.class, new Deserializer()).create();
        var field = gson.fromJson(file, AprilTagFieldLayout.class);
        if (name != null) fields.put(name, field);
        return field;
    }
    public static AprilTagFieldLayout get(String name) {
        var ret = fields.get(name);
        if (ret == null) System.err.println(String.format("Attepmted to load missing layout \"%s\"", name));
        return ret;
    }

    public static class Deserializer implements JsonDeserializer<AprilTagFieldLayout> {
        @Override
        public AprilTagFieldLayout deserialize(JsonElement json, Type type, JsonDeserializationContext context) throws JsonParseException {
            Shim sh = context.deserialize(json, Shim.class);
            var tags = sh.tags.stream().collect(Collectors.toMap(t -> t.ID, t -> t.pose.toPose()));
            return new AprilTagFieldLayout(sh.field.length, sh.field.width, tags);
        }
    }

    private static class Shim {
        List<TagShim> tags;
        FieldShim field;
    }
    private static class FieldShim {
        double length;
        double width;
    }
    private static class TagShim {
        int ID;
        PoseShim pose;
    }
    private static class PoseShim {
        TranslationShim translation;
        RotationShim rotation;

        Pose3d toPose() {
            return new Pose3d(
                translation.x,
                translation.y,
                translation.z,
                new Rotation3d(new Quaternion(
                    rotation.quaternion.W,
                    rotation.quaternion.X,
                    rotation.quaternion.Y,
                    rotation.quaternion.Z)));
        }
    }
    private static class TranslationShim {
        double x;
        double y;
        double z;
    }
    private static class RotationShim {
        QuaternionShim quaternion;
    }
    private static class QuaternionShim {
        double W;
        double X;
        double Y;
        double Z;
    }
}
