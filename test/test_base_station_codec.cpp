#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "oxebots_comms/base_station_codec.hpp"
#include "oxebots_interfaces/base_station_protocol_bp.h"

TEST(BaseStationCodecTest, RoundTripWithEmbeddedZeros)
{
    struct BaseRfTx tx = {};
    tx.seq = 0x1234;
    tx.target_robot_id = 3;
    tx.tx_flags = BS_TX_FLAG_EXPECT_REPLY;
    tx.payload_len = 8;
    tx.payload[0] = 0x00;
    tx.payload[1] = 0xAB;
    tx.payload[7] = 0x00;

    uint8_t raw[BYTES_LENGTH_BASE_RF_TX] = {};
    EncodeBaseRfTx(&tx, raw);

    const std::vector<uint8_t> frame =
        oxebots_comms::encode_base_frame(BS_MSG_RF_TX, raw, sizeof(raw));
    ASSERT_FALSE(frame.empty());
    EXPECT_EQ(frame.back(), 0x00U);

    oxebots_comms::BaseFrameDecoder decoder;
    std::vector<oxebots_comms::DecodedBaseFrame> decoded_frames;
    decoder.feed(frame.data(), frame.size(), [&](const oxebots_comms::DecodedBaseFrame& f) {
        decoded_frames.push_back(f);
    });

    ASSERT_EQ(decoded_frames.size(), 1U);
    EXPECT_EQ(decoded_frames[0].msg_type, BS_MSG_RF_TX);
    ASSERT_EQ(decoded_frames[0].payload.size(), BYTES_LENGTH_BASE_RF_TX);

    struct BaseRfTx out = {};
    DecodeBaseRfTx(&out, decoded_frames[0].payload.data());
    EXPECT_EQ(out.seq, tx.seq);
    EXPECT_EQ(out.target_robot_id, tx.target_robot_id);
    EXPECT_EQ(out.tx_flags, tx.tx_flags);
    EXPECT_EQ(out.payload_len, tx.payload_len);
    EXPECT_EQ(std::memcmp(out.payload, tx.payload, 31), 0);
}

TEST(BaseStationCodecTest, RejectsCorruptedCrcAndResyncs)
{
    struct BasePing ping = {};
    ping.seq = 42;
    uint8_t raw[BYTES_LENGTH_BASE_PING] = {};
    EncodeBasePing(&ping, raw);

    const auto good = oxebots_comms::encode_base_frame(BS_MSG_PING, raw, sizeof(raw));
    auto bad = good;
    bad[2] ^= 0x10U;

    oxebots_comms::BaseFrameDecoder decoder;
    int count = 0;
    decoder.feed(bad.data(), bad.size(), [&](const oxebots_comms::DecodedBaseFrame&) { ++count; });
    EXPECT_EQ(count, 0);

    decoder.feed(good.data(), good.size(), [&](const oxebots_comms::DecodedBaseFrame& f) {
        ++count;
        EXPECT_EQ(f.msg_type, BS_MSG_PING);
    });
    EXPECT_EQ(count, 1);
}

TEST(BaseStationCodecTest, ScanResultFull129ByteRoundTrip)
{
    struct BaseScanResult scan = {};
    scan.start_channel = 0;
    scan.count = 126;
    scan.best_channel = 114;
    for (int i = 0; i < 126; ++i) {
        scan.busy_pct[i] = static_cast<uint8_t>(i % 101);
    }

    uint8_t raw[BYTES_LENGTH_BASE_SCAN_RESULT] = {};
    EncodeBaseScanResult(&scan, raw);
    const auto frame = oxebots_comms::encode_base_frame(BS_MSG_SCAN_RESULT, raw, sizeof(raw));
    ASSERT_FALSE(frame.empty());

    oxebots_comms::BaseFrameDecoder decoder;
    std::vector<oxebots_comms::DecodedBaseFrame> frames;
    decoder.feed(frame.data(), frame.size(), [&](const oxebots_comms::DecodedBaseFrame& f) {
        frames.push_back(f);
    });
    ASSERT_EQ(frames.size(), 1U);
    ASSERT_EQ(frames[0].payload.size(), BYTES_LENGTH_BASE_SCAN_RESULT);

    struct BaseScanResult out = {};
    DecodeBaseScanResult(&out, frames[0].payload.data());
    EXPECT_EQ(out.start_channel, 0U);
    EXPECT_EQ(out.count, 126U);
    EXPECT_EQ(out.best_channel, 114U);
    EXPECT_EQ(std::memcmp(out.busy_pct, scan.busy_pct, 126), 0);
}
