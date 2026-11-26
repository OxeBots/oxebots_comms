#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace oxebots_comms
{

inline uint8_t crc8(const uint8_t* data, std::size_t len)
{
    uint8_t crc = 0;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

inline std::size_t cobs_encode(const uint8_t* in, std::size_t len, uint8_t* out)
{
    std::size_t written = 0;
    std::size_t code_index = 0;
    uint8_t code = 1;
    ++written;
    for (std::size_t i = 0; i < len; ++i) {
        if (in[i] != 0U) {
            out[written++] = in[i];
            if (++code == 0xFFU) {
                out[code_index] = 0xFFU;
                code = 1;
                code_index = written;
                ++written;
            }
        } else {
            out[code_index] = code;
            code = 1;
            code_index = written;
            ++written;
        }
    }
    out[code_index] = code;
    return written;
}

inline std::size_t cobs_decode(const uint8_t* in, std::size_t len, uint8_t* out, std::size_t out_cap)
{
    std::size_t written = 0;
    std::size_t i = 0;
    while (i < len) {
        const uint8_t code = in[i++];
        if (code == 0U) return 0;
        for (uint8_t j = 1; j < code; ++j) {
            if (i >= len || written >= out_cap) return 0;
            out[written++] = in[i++];
        }
        if (code < 0xFFU && i < len) {
            if (written >= out_cap) return 0;
            out[written++] = 0U;
        }
    }
    return written;
}

inline std::vector<uint8_t> encode_base_frame(
    uint8_t msg_type, const uint8_t* payload, std::size_t payload_len)
{
    if (payload == nullptr || payload_len == 0 || payload_len > 250) {
        return {};
    }
    std::vector<uint8_t> staged(payload_len + 2);
    staged[0] = msg_type;
    std::memcpy(staged.data() + 1, payload, payload_len);
    staged[payload_len + 1] = crc8(staged.data(), payload_len + 1);

    std::vector<uint8_t> out(staged.size() + staged.size() / 254 + 2);
    const std::size_t encoded_len = cobs_encode(staged.data(), staged.size(), out.data());
    out[encoded_len] = 0x00U;
    out.resize(encoded_len + 1);
    return out;
}

struct DecodedBaseFrame
{
    uint8_t msg_type = 0;
    std::vector<uint8_t> payload;
};

class BaseFrameDecoder
{
public:
    static constexpr std::size_t MAX_ACCUM_BYTES = 264;

    void reset()
    {
        accum_.clear();
        overflow_ = false;
    }

    template <typename Callback>
    void feed(const uint8_t* data, std::size_t len, Callback&& on_frame)
    {
        for (std::size_t i = 0; i < len; ++i) {
            const uint8_t b = data[i];
            if (b == 0x00U) {
                if (overflow_) {
                    overflow_ = false;
                    accum_.clear();
                } else if (!accum_.empty()) {
                    DecodedBaseFrame frame;
                    if (decode_accum(&frame)) {
                        on_frame(frame);
                    }
                    accum_.clear();
                }
                continue;
            }
            if (overflow_) continue;
            if (accum_.size() >= MAX_ACCUM_BYTES) {
                accum_.clear();
                overflow_ = true;
                continue;
            }
            accum_.push_back(b);
        }
    }

private:
    bool decode_accum(DecodedBaseFrame* out)
    {
        uint8_t decoded[MAX_ACCUM_BYTES];
        const std::size_t n = cobs_decode(accum_.data(), accum_.size(), decoded, sizeof(decoded));
        if (n < 3) return false;
        if (crc8(decoded, n - 1) != decoded[n - 1]) return false;
        out->msg_type = decoded[0];
        out->payload.assign(decoded + 1, decoded + n - 1);
        return true;
    }

    std::vector<uint8_t> accum_;
    bool overflow_ = false;
};

}  // namespace oxebots_comms
