package frc.vision.load;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import frc.vision.camera.CameraBase;
import frc.vision.camera.CameraConfig;
import java.lang.reflect.Type;
import java.io.IOException;
import java.io.FileReader;
import java.io.Reader;
import java.time.LocalDateTime;
import java.util.HashMap;
import java.util.Map;

// A camera that can be loaded from the configuration file.
public class CameraLoader {
    protected static HashMap<String, CameraFactory> types = new HashMap<String, CameraFactory>();
    protected static HashMap<String, WrappedConfig> configs = new HashMap<String, WrappedConfig>();
    private static boolean configInitialized = false;

    protected static class WrappedConfig {
        CameraConfig inner;
        public WrappedConfig(CameraConfig inner) {
            this.inner = inner;
        }
    }

    protected static class CustomDeserializer implements JsonDeserializer<WrappedConfig> {
        @Override
        public WrappedConfig deserialize(JsonElement json, Type type, JsonDeserializationContext context) throws JsonParseException {
            JsonObject obj = json.getAsJsonObject();
            JsonElement elem = obj.get("type");
            if (elem == null) {
                CameraConfig cfg = context.deserialize(json, CameraConfig.class);
                return new WrappedConfig(cfg);
            }
            String ty = elem.getAsString();
            CameraFactory factory = types.get(ty);
            CameraConfig cfg = context.deserialize(json, factory.configType());
            return new WrappedConfig(cfg);
        }
    }

    protected CameraLoader() {}

    // Initialize configs from a file, in JSON.
    public static void initConfig(Reader file) throws JsonParseException, JsonIOException {
        if (configInitialized) {
            System.err.println("Calling CameraLoader.initConfig() when already initialized does nothing");
            return;
        }
        GsonBuilder builder = new GsonBuilder().registerTypeAdapter(WrappedConfig.class, new CustomDeserializer());
        for (CameraFactory fac : types.values()) {
            fac.modifyBuilder(builder);
        }
        Gson gson = builder.create();
        configs = gson.fromJson(file, new TypeToken<HashMap<String, WrappedConfig>>() {}.getType());
        
        for (Map.Entry<String, WrappedConfig> entry : configs.entrySet()) {
            if (entry.getKey() == "default") continue;
            WrappedConfig default_ = configs.get("default");
            if (default_ != null) entry.getValue().inner.updateFrom(default_.inner);
        }
        configInitialized = true;
    }
    public static void initConfig() throws JsonParseException, IOException {
        var res = CameraLoader.class
            .getClassLoader()
            .getResource("cameras.json");
        if (res == null) return;
        initConfig(new FileReader(res.getFile()));
    }

    // Register a factory to create cameras.
    public static void registerFactory(CameraFactory factory) {
        types.put(factory.typeName(), factory);
    }

    // Load a camera with the given name.
    // TODO: give better exceptions.
    public static CameraBase load(String name, LocalDateTime date) throws IOException {
        WrappedConfig wcfg = configs.get(name);
        CameraConfig cfg = wcfg.inner;
        CameraFactory factory = types.get(cfg.type);
        return factory.create(name, cfg, date);
    }
    // Load a camera with the given name.
    // TODO: give better exceptions.
    public static CameraBase load(String name) throws IOException {
        return load(name, LocalDateTime.now());
    }
}
