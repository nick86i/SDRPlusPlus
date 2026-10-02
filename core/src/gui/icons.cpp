#include <gui/icons.h>
#include <stdint.h>
#include <config.h>

#define STB_IMAGE_IMPLEMENTATION
#include <imgui/stb_image.h>
#include <filesystem>
#include <vector>
#include <utils/flog.h>

namespace icons {
    ImTextureID LOGO;
    ImTextureID PLAY;
    ImTextureID STOP;
    ImTextureID MENU;
    ImTextureID MUTED;
    ImTextureID UNMUTED;
    ImTextureID NORMAL_TUNING;
    ImTextureID CENTER_TUNING;
    ImTextureID STICKY_TUNING;

    static ImTextureID loadPushpinTexture() {
        constexpr int size = 256;
        std::vector<uint8_t> pixels(size * size * 4, 0);
        auto setPixel = [&](int x, int y) {
            if (x < 0 || x >= size || y < 0 || y >= size) { return; }
            size_t pos = ((size_t)y * size + x) * 4;
            pixels[pos] = pixels[pos + 1] = pixels[pos + 2] = pixels[pos + 3] = 255;
        };

        // Upright pushpin: rounded cap, narrow stem, and pointed tip.
        for (int y = 42; y <= 91; y++) {
            for (int x = 58; x <= 198; x++) {
                int dx = (x < 73) ? 73 - x : ((x > 183) ? x - 183 : 0);
                int dy = (y < 57) ? 57 - y : ((y > 76) ? y - 76 : 0);
                if ((dx * dx) + (dy * dy) <= 15 * 15) { setPixel(x, y); }
            }
        }
        for (int y = 82; y <= 137; y++) {
            int inset = (y - 82) / 2;
            for (int x = 76 + inset; x <= 180 - inset; x++) { setPixel(x, y); }
        }
        for (int y = 126; y <= 151; y++) {
            for (int x = 55; x <= 201; x++) { setPixel(x, y); }
        }
        for (int y = 151; y <= 220; y++) {
            int halfWidth = (std::max)(2, 13 - ((y - 151) / 6));
            for (int x = 128 - halfWidth; x <= 128 + halfWidth; x++) { setPixel(x, y); }
        }

        GLuint texId;
        glGenTextures(1, &texId);
        glBindTexture(GL_TEXTURE_2D, texId);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, size, size, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        return (ImTextureID)(uintptr_t)texId;
    }

    GLuint loadTexture(std::string path) {
        int w, h, n;
        stbi_uc* data = stbi_load(path.c_str(), &w, &h, &n, 0);
        GLuint texId;
        glGenTextures(1, &texId);
        glBindTexture(GL_TEXTURE_2D, texId);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, (uint8_t*)data);
        stbi_image_free(data);
        return texId;
    }

    bool load(std::string resDir) {
        if (!std::filesystem::is_directory(resDir)) {
            flog::error("Invalid resource directory: {0}", resDir);
            return false;
        }

        LOGO = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/sdrpp.png");
        PLAY = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/play.png");
        STOP = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/stop.png");
        MENU = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/menu.png");
        MUTED = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/muted.png");
        UNMUTED = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/unmuted.png");
        NORMAL_TUNING = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/normal_tuning.png");
        CENTER_TUNING = (ImTextureID)(uintptr_t)loadTexture(resDir + "/icons/center_tuning.png");
        STICKY_TUNING = loadPushpinTexture();

        return true;
    }
}
