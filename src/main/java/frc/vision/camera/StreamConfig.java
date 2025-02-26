package frc.vision.camera;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import java.lang.reflect.Type;

public class StreamConfig {
    private static class Shim {
        int port = 1181;
        String address;
        String name;
        int fps = 30;
    }
    public int port = 1181;
    public String address;
    public String name;
    public int fps = 30;

    public static class Deserializer implements JsonDeserializer<StreamConfig> {
        @Override
        public StreamConfig deserialize(JsonElement json, Type type, JsonDeserializationContext context) throws JsonParseException {
            StreamConfig out = new StreamConfig();
            if (json instanceof JsonPrimitive) {
                JsonPrimitive prim = (JsonPrimitive)json;
                if (prim.isNumber()) {
                    out.port = prim.getAsInt();
                    return out;
                }
                if (prim.isString()) {
                    String addr = prim.getAsString();
                    int idx = addr.lastIndexOf(":");
                    if (idx >= 0) {
                        try {
                            String portStr = addr.substring(idx + 1);
                            out.port = Integer.valueOf(portStr);
                            addr = addr.substring(0, idx);
                        } catch (NumberFormatException ex) {}
                    }
                    out.address = addr;
                    return out;
                }
            }
            Shim sh = context.deserialize(json, new TypeToken<Shim>() {}.getType());
            out.port = sh.port;
            out.address = sh.address;
            out.name = sh.name;
            out.fps = sh.fps;
            return out;
        }
    }
}
