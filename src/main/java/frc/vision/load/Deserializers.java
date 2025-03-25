package frc.vision.load;

import com.google.gson.*;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.lang.reflect.Type;

public final class Deserializers {
    private Deserializers() {}

    public static class TransformDeserializer implements JsonDeserializer<Transform3d> {
        private static class Shim {
            public double x;
            public double y;
            public double z;
            public double yaw;
            public double pitch;
            public double roll;
        }
        @Override
        public Transform3d deserialize(JsonElement json, Type type, JsonDeserializationContext context) throws JsonParseException {
            Shim shim = context.deserialize(json, Shim.class);
            var rot = new Rotation3d(shim.roll, shim.pitch, shim.roll);
            return new Transform3d(shim.x, shim.y, shim.z, rot);
        }
    }
}
