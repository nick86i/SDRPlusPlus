#include "mot_image.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include <imgui/stb_image.h>

#include <algorithm>
#include <cstring>

bool decodeMotImage(const std::vector<std::uint8_t>& encoded,
                    std::vector<std::uint8_t>& canvas,
                    int canvasWidth, int canvasHeight,
                    int& imageWidth, int& imageHeight) {
    if (encoded.empty() || canvasWidth <= 0 || canvasHeight <= 0) return false;

    int channels = 0;
    stbi_uc* pixels = stbi_load_from_memory(encoded.data(), static_cast<int>(encoded.size()),
                                             &imageWidth, &imageHeight, &channels, 4);
    if (!pixels || imageWidth <= 0 || imageHeight <= 0) {
        if (pixels) stbi_image_free(pixels);
        return false;
    }

    canvas.assign(static_cast<std::size_t>(canvasWidth) * canvasHeight * 4, 0);
    const float scale = (std::min)(static_cast<float>(canvasWidth) / imageWidth,
                                   static_cast<float>(canvasHeight) / imageHeight);
    const int drawWidth = (std::max)(1, static_cast<int>(imageWidth * scale));
    const int drawHeight = (std::max)(1, static_cast<int>(imageHeight * scale));
    const int offsetX = (canvasWidth - drawWidth) / 2;
    const int offsetY = (canvasHeight - drawHeight) / 2;

    for (int y = 0; y < drawHeight; ++y) {
        const int sourceY = (std::min)(imageHeight - 1, y * imageHeight / drawHeight);
        for (int x = 0; x < drawWidth; ++x) {
            const int sourceX = (std::min)(imageWidth - 1, x * imageWidth / drawWidth);
            const auto* source = pixels + (static_cast<std::size_t>(sourceY) * imageWidth + sourceX) * 4;
            auto* destination = canvas.data() +
                (static_cast<std::size_t>(offsetY + y) * canvasWidth + offsetX + x) * 4;
            std::memcpy(destination, source, 4);
        }
    }

    stbi_image_free(pixels);
    return true;
}
