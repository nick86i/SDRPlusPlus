#include "DVB_T2/LDPC/dvb_t2_tables.hh"
#include "DVB_T2/LDPC/algorithms.hh"
#include "DVB_T2/LDPC/layered_decoder.hh"
#include <bch/bch.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

constexpr int DVB_T2_TABLE_NORMAL_C2_3::DEG[];
constexpr int DVB_T2_TABLE_NORMAL_C2_3::LEN[];
constexpr int DVB_T2_TABLE_NORMAL_C2_3::POS[];

namespace {
constexpr int kFecBits = 64800;
constexpr int kInformationBits = 43200;
constexpr int kParityBits = kFecBits - kInformationBits;
constexpr int kLanes = 32;
constexpr int kQ = 60;
using LdpcSimd = SIMD<int8_t, kLanes>;

struct TraceHeader {
    uint32_t magic;
    uint32_t version;
    int32_t fecBits;
    int32_t informationBits;
    int32_t fecBlock;
    int32_t dynamicFrame;
    int32_t modulation;
    int32_t codeRate;
    int32_t rotation;
    float snrDb;
};

int countUnsatisfied(LDPC<DVB_T2_TABLE_NORMAL_C2_3>& table,
                     const std::vector<LdpcSimd>& words) {
    std::vector<uint8_t> checks(kParityBits, 0);
    table.first_bit();
    for (int bit = 0; bit < kInformationBits; ++bit) {
        const int degree = table.bit_deg();
        const int* positions = table.acc_pos();
        if (words[bit].v[0] < 0) {
            for (int edge = 0; edge < degree; ++edge) checks[positions[edge]] ^= 1;
        }
        table.next_bit();
    }
    int unsatisfied = 0;
    for (int group = 0; group < kQ; ++group) {
        for (int row = 0; row < 360; ++row) {
            const int checkIndex = kQ * row + group;
            int syndrome = checks[checkIndex];
            syndrome ^= words[kInformationBits + kQ * row + group].v[0] < 0;
            if (group != 0)
                syndrome ^= words[kInformationBits + kQ * row + group - 1].v[0] < 0;
            else if (row != 0)
                syndrome ^= words[kInformationBits + kQ * (row - 1) + kQ - 1].v[0] < 0;
            unsatisfied += syndrome;
        }
    }
    return unsatisfied;
}

int tryBch(const std::vector<LdpcSimd>& words) {
    static const bch::galois_field<uint32_t> field(
        bch::gf2_poly<uint32_t>(0b10000000000101101u));
    static const bch::bch_codec<uint32_t, bch::bitset256_t> decoder(&field, 10, 43200);
    std::array<uint8_t, 5400> codeword{};
    std::array<uint8_t, 5380> decoded{};
    for (int bit = 0; bit < 43200; ++bit) {
        if (words[bit].v[0] < 0)
            codeword[bit / 8] |= static_cast<uint8_t>(0x80u >> (bit & 7));
    }
    return decoder.decode(codeword.data(), decoded.data(), true);
}

std::vector<LdpcSimd> makeWords(const std::vector<int8_t>& source, double scale) {
    std::vector<LdpcSimd> words(kFecBits);
    auto scaled = [scale](int8_t value) {
        const int result = static_cast<int>(std::lround(static_cast<double>(value) * scale));
        return static_cast<int8_t>((std::max)(-127, (std::min)(127, result)));
    };
    for (int lane = 0; lane < kLanes; ++lane) {
        for (int i = 0; i < kInformationBits; ++i) words[i].v[lane] = scaled(source[i]);
        for (int group = 0; group < kQ; ++group) {
            for (int row = 0; row < 360; ++row) {
                words[kInformationBits + kQ * row + group].v[lane] =
                    scaled(source[kInformationBits + 360 * group + row]);
            }
        }
    }
    return words;
}

template <typename Algorithm>
void runCase(const char* name, LDPC<DVB_T2_TABLE_NORMAL_C2_3>& table,
             const std::vector<int8_t>& source, double scale, int iterations) {
    auto words = makeWords(source, scale);
    LDPCDecoder<LdpcSimd, Algorithm> decoder;
    decoder.init(&table);
    const int remaining = decoder(words.data(), words.data() + kInformationBits, iterations, 1);
    const int unsatisfied = countUnsatisfied(table, words);
    const int bch = unsatisfied == 0 ? 0 : tryBch(words);
    std::cout << name << ',' << scale << ',' << iterations << ',' << remaining << ','
              << unsatisfied << ',' << bch << '\n';
}
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: dvbt2_ldpc_replay <dvbt2_residual_fec.bin>\n";
        return 2;
    }
    std::ifstream input(argv[1], std::ios::binary);
    TraceHeader header{};
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!input || header.magic != 0x45463252u || header.version != 1 ||
        header.fecBits != kFecBits || header.informationBits != kInformationBits) {
        std::cerr << "invalid residual FEC trace\n";
        return 1;
    }
    std::vector<int8_t> source(kFecBits);
    input.read(reinterpret_cast<char*>(source.data()), source.size());
    if (!input) {
        std::cerr << "truncated residual FEC trace\n";
        return 1;
    }

    using Normal = gnr::NormalUpdate<LdpcSimd>;
    using SelfCorrected = gnr::SelfCorrectedUpdate<LdpcSimd>;
    using Offset = gnr::OffsetMinSumAlgorithm<LdpcSimd, Normal, 2>;
    using MinSum = gnr::MinSumAlgorithm<LdpcSimd, Normal>;
    using OffsetSelfCorrected = gnr::OffsetMinSumAlgorithm<LdpcSimd, SelfCorrected, 2>;
    using MinSumSelfCorrected = gnr::MinSumAlgorithm<LdpcSimd, SelfCorrected>;
    LDPC<DVB_T2_TABLE_NORMAL_C2_3> table;

    std::cout << "# block=" << header.fecBlock << " frame=" << header.dynamicFrame
              << " snr_db=" << header.snrDb << '\n';
    std::cout << "algorithm,scale,iterations,trials_remaining,unsatisfied,bch_corrected\n";
    constexpr std::array<double, 10> scales{0.25, 0.33, 0.4, 0.6, 0.66, 0.8, 1.75, 2.0, 2.5, 3.0};
    constexpr std::array<int, 2> iterationCounts{25, 35};
    for (double scale : scales) {
        for (int iterations : iterationCounts) {
            runCase<Offset>("offset", table, source, scale, iterations);
            runCase<MinSum>("minsum", table, source, scale, iterations);
            runCase<OffsetSelfCorrected>("offset-self-corrected", table, source, scale, iterations);
            runCase<MinSumSelfCorrected>("minsum-self-corrected", table, source, scale, iterations);
        }
    }
    return 0;
}
