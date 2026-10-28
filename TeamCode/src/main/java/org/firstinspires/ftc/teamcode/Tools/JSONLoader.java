
package org.firstinspires.ftc.teamcode.Tools;

import android.content.Context;

import org.json.JSONObject;

import java.io.InputStream;
import java.io.ByteArrayOutputStream;
import java.nio.charset.StandardCharsets;

public class JSONLoader {

    private final JSONObject json;

    public JSONLoader(Context context, String name) {
        String filePath = "paths/" + name;

        try (InputStream input = context.getAssets().open(filePath);
             ByteArrayOutputStream output = new ByteArrayOutputStream()) {

            byte[] buffer = new byte[4096];
            int length;

            while ((length = input.read(buffer)) != -1) {
                output.write(buffer, 0, length);
            }

            String content = new String(
                    output.toByteArray(),
                    StandardCharsets.UTF_8
            );

            // Parse .polarpath exactly like JSON
            json = new JSONObject(content);

        } catch (Exception e) {
            throw new RuntimeException(
                    "Failed to load path: " + filePath, e
            );
        }
    }

    public JSONObject getJSON() {
        return json;
    }
}
