#pragma once

#include <cstdint>
#include <vector>

bool decodeMotImage(const std::vector<std::uint8_t>& encoded,
                    std::vector<std::uint8_t>& canvas,
                    int canvasWidth, int canvasHeight,
                    int& imageWidth, int& imageHeight);
