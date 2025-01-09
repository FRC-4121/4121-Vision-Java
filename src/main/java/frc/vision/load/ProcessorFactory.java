package frc.vision.load;

import frc.vision.process.ProcessorConfig;
import frc.vision.process.VisionProcessor;

// A camera that can be loaded, for use with CameraLoader.
public abstract class ProcessorFactory {
    // The type name that this should be referred to in the config file.
    public abstract String typeName();
    // The configuration type to deserialize to.
    public Class<? extends ProcessorConfig> configType() {
        return ProcessorConfig.class;
    }
    // Create this processor, given the name and configuration.
    // The configuration type is guaranteed to have the same type as the specified config type.
    public abstract VisionProcessor create(String name, ProcessorConfig cfg);
}
