package frc.vision.load;

import com.google.gson.GsonBuilder;
import frc.vision.camera.CameraBase;
import frc.vision.camera.CameraConfig;
import java.io.IOException;
import java.time.LocalDateTime;

// A camera that can be loaded, for use with CameraLoader.
public abstract class CameraFactory {
    // The type name that this should be referred to in the config file.
    public abstract String typeName();
    // The configuration type to deserialize to, defaults to just CameraConfig.
    public Class<? extends CameraConfig> configType() {
        return CameraConfig.class;
    }
    // Modify the builder, used to add a custom deserializer.
    public void modifyBuilder(GsonBuilder builder) {}
    // Create this, given the name and camera configuration.
    // The configuration type is guaranteed to have the same type as the specified config type.
    public abstract CameraBase create(String name, CameraConfig cfg, LocalDateTime date) throws IOException;
}
