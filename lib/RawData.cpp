#include <motioncam/RawData.hpp>
#include <vector>
#include <cstring>

namespace motioncam
{
    namespace raw
    {

        // Constants used by the decoder:
        const int ENCODING_BLOCK = 64;
        const int HEADER_LENGTH = 2;
        const int METADATA_OFFSET = 16;

        // The number of bytes read for each bits value.
        constexpr int ENCODING_BLOCK_LENGTH[] = {
            0,
            8,  // Decode1 uses 8 bytes
            8,  // Decode2: each sub–block uses 8 bytes (called twice)
            24, // Decode3 uses 24 bytes (3×8)
            8,  // Decode4_One uses 8 bytes (called 4×)
            40, // Decode5 uses 40 bytes (5×8)
            48, // Decode6 uses 48 bytes (6×8)
            0,  // not used (bits value 7 falls back to 8)
            8,  // Decode8 uses 64 bytes (8×8) – see below
            0,  // bits value 9 falls back to Decode10 below
            80, // Decode10 uses 80 bytes (10×8)
            0   // bits value 16 uses Decode16 (which uses 16 bytes per ONE call; 8×2=16×? see below)
        };

        // Decode1:
        // Data in one 8–byte chunk is “expanded” into 8 rows of 8 values.
        inline const uint8_t *Decode1(uint16_t *output, const uint8_t *input)
        {
            // Read 8 bytes from input:
            uint8_t p[8];
            for (int i = 0; i < 8; i++)
                p[i] = input[i];

            // For each bit 0 ... 7, extract that bit from all 8 bytes.
            for (int i = 0; i < 8; i++)
            {
                output[i] = p[i] & 0x01;
                output[8 + i] = (p[i] >> 1) & 0x01;
                output[16 + i] = (p[i] >> 2) & 0x01;
                output[24 + i] = (p[i] >> 3) & 0x01;
                output[32 + i] = (p[i] >> 4) & 0x01;
                output[40 + i] = (p[i] >> 5) & 0x01;
                output[48 + i] = (p[i] >> 6) & 0x01;
                output[56 + i] = (p[i] >> 7) & 0x01;
            }

            return input + ENCODING_BLOCK_LENGTH[1];
        }

        // Decode2_One extracts 4 groups (each group 8 values)
        inline const uint8_t *Decode2_One(uint16_t *output, const uint8_t *input)
        {
            uint8_t p[8];
            for (int i = 0; i < 8; i++)
                p[i] = input[i];

            for (int i = 0; i < 8; i++)
            {
                output[i] = p[i] & 0x03; // lower 2 bits
                output[8 + i] = (p[i] >> 2) & 0x03;
                output[16 + i] = (p[i] >> 4) & 0x03;
                output[24 + i] = (p[i] >> 6) & 0x03;
            }

            return input + 8;
        }

        inline const uint8_t *Decode2(uint16_t *output, const uint8_t *input)
        {
            input = Decode2_One(output, input);
            input = Decode2_One(output + 32, input);
            return input;
        }

        // Decode3 works on three 8–byte chunks.
        // This decodes eight numbers per row (r0..r7). Some bits come from different parts.
        inline const uint8_t *Decode3(uint16_t *output, const uint8_t *input)
        {
            uint8_t p0[8], p1[8], p2[8];
            for (int i = 0; i < 8; i++)
            {
                p0[i] = input[i];
                p1[i] = input[8 + i];
                p2[i] = input[16 + i];
            }

            // For each index, compute each result:
            for (int i = 0; i < 8; i++)
            {
                uint16_t r0 = p0[i] & 0x07;         // mask 0x07 = 0b00000111
                uint16_t r1 = (p0[i] >> 3) & 0x07;  // next 3 bits
                uint16_t _r2 = (p0[i] >> 6) & 0x03; // lower 2 from top 2 bits of p0
                uint16_t r3 = p1[i] & 0x07;
                uint16_t r4 = (p1[i] >> 3) & 0x07;
                uint16_t _r5 = (p1[i] >> 6) & 0x03;
                uint16_t r6 = p2[i] & 0x07;
                uint16_t r7 = (p2[i] >> 3) & 0x07;
                // Now restore the upper bits:
                uint16_t add_r2 = ((p2[i] >> 6) & 0x01) << 2;
                uint16_t add_r5 = ((p2[i] >> 7) & 0x01) << 2;
                uint16_t r2 = _r2 | add_r2;
                uint16_t r5 = _r5 | add_r5;

                output[i] = r0;
                output[8 + i] = r1;
                output[16 + i] = r2;
                output[24 + i] = r3;
                output[32 + i] = r4;
                output[40 + i] = r5;
                output[48 + i] = r6;
                output[56 + i] = r7;
            }

            return input + ENCODING_BLOCK_LENGTH[3];
        }

        // Decode4_One decodes one 8-byte chunk into two groups (8 numbers each)
        inline const uint8_t *Decode4_One(uint16_t *output, const uint8_t *input)
        {
            uint8_t p[8];
            for (int i = 0; i < 8; i++)
                p[i] = input[i];

            for (int i = 0; i < 8; i++)
            {
                output[i] = p[i] & 0x0F;
                output[8 + i] = (p[i] >> 4) & 0x0F;
            }

            return input + 8;
        }

        inline const uint8_t *Decode4(uint16_t *output, const uint8_t *input)
        {
            input = Decode4_One(output, input);
            input = Decode4_One(output + 16, input);
            input = Decode4_One(output + 32, input);
            input = Decode4_One(output + 48, input);
            return input;
        }

        // Decode5 decodes 5 successive 8–byte chunks.
        inline const uint8_t *Decode5(uint16_t *output, const uint8_t *input)
        {
            uint8_t p0[8], p1[8], p2[8], p3[8], p4[8];
            for (int i = 0; i < 8; i++)
            {
                p0[i] = input[i];
                p1[i] = input[8 + i];
                p2[i] = input[16 + i];
                p3[i] = input[24 + i];
                p4[i] = input[32 + i];
            }

            for (int i = 0; i < 8; i++)
            {
                uint16_t r0 = p0[i] & 0x1F;
                uint16_t r1 = p1[i] & 0x1F;
                uint16_t r2 = p2[i] & 0x1F;
                uint16_t r3 = p3[i] & 0x1F;
                uint16_t r4 = p4[i] & 0x1F;
                uint16_t r5 = ((p0[i] >> 5) & 0x07) | (((p3[i] >> 5) & 0x03) << 3);
                uint16_t r6 = ((p1[i] >> 5) & 0x07) | (((p4[i] >> 5) & 0x03) << 3);
                uint16_t tmp0 = (p2[i] >> 5) & 0x07;
                uint16_t tmp1 = tmp0 | (((p3[i] >> 7) & 0x01) << 3);
                uint16_t r7 = tmp1 | (((p4[i] >> 7) & 0x01) << 4);

                output[i] = r0;
                output[8 + i] = r1;
                output[16 + i] = r2;
                output[24 + i] = r3;
                output[32 + i] = r4;
                output[40 + i] = r5;
                output[48 + i] = r6;
                output[56 + i] = r7;
            }

            return input + ENCODING_BLOCK_LENGTH[5];
        }

        // Decode6 decodes 6 successive 8-byte chunks.
        inline const uint8_t *Decode6(uint16_t *output, const uint8_t *input)
        {
            uint8_t p0[8], p1[8], p2[8], p3[8], p4[8], p5[8];
            for (int i = 0; i < 8; i++)
            {
                p0[i] = input[i];
                p1[i] = input[8 + i];
                p2[i] = input[16 + i];
                p3[i] = input[24 + i];
                p4[i] = input[32 + i];
                p5[i] = input[40 + i];
            }

            for (int i = 0; i < 8; i++)
            {
                uint16_t r0 = p0[i] & 0x3F;
                uint16_t r1 = p1[i] & 0x3F;
                uint16_t r2 = p2[i] & 0x3F;
                uint16_t r3 = p3[i] & 0x3F;
                uint16_t r4 = p4[i] & 0x3F;
                uint16_t r5 = p5[i] & 0x3F;
                // Note: the original SIMD code combined two identical shifts from p1; we follow that logic:
                uint16_t r6 = ((p0[i] >> 6) & 0x03) | (((p1[i] >> 6) & 0x03) << 2) | (((p1[i] >> 6) & 0x03) << 2) | (((p2[i] >> 6) & 0x03) << 4);
                uint16_t r7 = ((p3[i] >> 6) & 0x03) | (((p4[i] >> 6) & 0x03) << 2) | (((p5[i] >> 6) & 0x03) << 4);

                output[i] = r0;
                output[8 + i] = r1;
                output[16 + i] = r2;
                output[24 + i] = r3;
                output[32 + i] = r4;
                output[40 + i] = r5;
                output[48 + i] = r6;
                output[56 + i] = r7;
            }

            return input + ENCODING_BLOCK_LENGTH[6];
        }

        // Decode8_One: simply copies eight bytes (each byte becomes a uint16_t)
        inline const uint8_t *Decode8_One(uint16_t *output, const uint8_t *input)
        {
            for (int i = 0; i < 8; i++)
            {
                output[i] = input[i];
            }
            return input + 8;
        }

        // Decode8 calls Decode8_One eight times.
        inline const uint8_t *Decode8(uint16_t *output, const uint8_t *input)
        {
            input = Decode8_One(output, input);
            input = Decode8_One(output + 8, input);
            input = Decode8_One(output + 16, input);
            input = Decode8_One(output + 24, input);
            input = Decode8_One(output + 32, input);
            input = Decode8_One(output + 40, input);
            input = Decode8_One(output + 48, input);
            input = Decode8_One(output + 56, input);
            return input;
        }

        // Decode10 decodes 10 successive 8-byte chunks.
        inline const uint8_t *Decode10(uint16_t *output, const uint8_t *input)
        {
            uint8_t p[10][8];
            for (int j = 0; j < 10; j++)
            {
                for (int i = 0; i < 8; i++)
                {
                    p[j][i] = input[j * 8 + i];
                }
            }

            // For first four outputs, combine bits from p0..p4:
            for (int i = 0; i < 8; i++)
            {
                uint16_t r0 = (p[0][i] & 0xFF) | ((p[4][i] & 0x03) << 8);
                uint16_t r1 = (p[1][i] & 0xFF) | ((p[4][i] & 0x0C) << 6);
                uint16_t r2 = (p[2][i] & 0xFF) | ((p[4][i] & 0x30) << 4);
                uint16_t r3 = (p[3][i] & 0xFF) | ((p[4][i] & 0xC0) << 2);

                output[i] = r0;
                output[8 + i] = r1;
                output[16 + i] = r2;
                output[24 + i] = r3;
            }

            // For next four outputs, combine bits from p5..p9:
            for (int i = 0; i < 8; i++)
            {
                uint16_t r4 = (p[5][i] & 0xFF) | ((p[9][i] & 0x03) << 8);
                uint16_t r5 = (p[6][i] & 0xFF) | ((p[9][i] & 0x0C) << 6);
                uint16_t r6 = (p[7][i] & 0xFF) | ((p[9][i] & 0x30) << 4);
                uint16_t r7 = (p[8][i] & 0xFF) | ((p[9][i] & 0xC0) << 2);

                output[32 + i] = r4;
                output[40 + i] = r5;
                output[48 + i] = r6;
                output[56 + i] = r7;
            }

            return input + ENCODING_BLOCK_LENGTH[10];
        }

        // Decode16_ONE decodes 16 bytes into 8 uint16_t values.
        inline const uint8_t *Decode16_ONE(uint16_t *output, const uint8_t *input)
        {
            const uint16_t *input16 = reinterpret_cast<const uint16_t *>(input);
            for (int i = 0; i < 8; i++)
                output[i] = input16[i];
            return input + 16;
        }

        inline const uint8_t *Decode16(uint16_t *output, const uint8_t *input)
        {
            input = Decode16_ONE(output, input);
            input = Decode16_ONE(output + 8, input);
            input = Decode16_ONE(output + 16, input);
            input = Decode16_ONE(output + 24, input);
            input = Decode16_ONE(output + 32, input);
            input = Decode16_ONE(output + 40, input);
            input = Decode16_ONE(output + 48, input);
            input = Decode16_ONE(output + 56, input);
            return input;
        }

        // DecodeBlock selects the appropriate decoder based on the bits value.
        inline size_t DecodeBlock(uint16_t *output, const uint16_t bits, const uint8_t *input, const size_t offset, const size_t len)
        {
            // Do not decode if past end of input:
            if (offset + ENCODING_BLOCK_LENGTH[bits] > len)
                return len - offset;

            input += offset;

            switch (bits)
            {
            case 0:
                std::memset(output, 0, sizeof(uint16_t) * ENCODING_BLOCK);
                break;
            case 1:
                Decode1(output, input);
                break;
            case 2:
                Decode2(output, input);
                break;
            case 3:
                Decode3(output, input);
                break;
            case 4:
                Decode4(output, input);
                break;
            case 5:
                Decode5(output, input);
                break;
            case 6:
                Decode6(output, input);
                break;
            case 7:
            case 8:
                Decode8(output, input);
                break;
            case 9:
            case 10:
                Decode10(output, input);
                break;
            default:
            case 16:
                Decode16(output, input);
                break;
            }

            return ENCODING_BLOCK_LENGTH[bits];
        }

        // DecodeMetadata decodes a series of blocks (each with header + block data)
        inline size_t DecodeMetadata(const uint8_t *input, size_t offset, const size_t len, std::vector<uint16_t> &outMetadata)
        {
            uint32_t numBlocks =
                static_cast<uint32_t>(input[offset]) |
                (static_cast<uint32_t>(input[offset + 1]) << 8) |
                (static_cast<uint32_t>(input[offset + 2]) << 16) |
                (static_cast<uint32_t>(input[offset + 3]) << 24);

            outMetadata.resize(numBlocks);
            offset += 4;

            uint8_t bits;
            uint16_t reference;

            // Decode bits – each block has a header (2 bytes) that encodes bits and reference.
            uint16_t *data = outMetadata.data();

            for (uint32_t i = 0; i < numBlocks; i += ENCODING_BLOCK)
            {
                // Decode header (2 bytes): high nibble of first byte, and then combine low nibble with next byte.
                bits = (input[offset] >> 4) & 0x0F;
                reference = ((input[offset] & 0x0F) << 8) | input[offset + 1];
                offset += HEADER_LENGTH;

                offset += DecodeBlock(data, bits, input, offset, len);

                // Add the reference correction to each element of the current block.
                for (int x = 0; x < ENCODING_BLOCK; x++)
                    data[x] += reference;
                data += ENCODING_BLOCK;
            }

            return offset;
        }

        // ReadMetadataHeader reads image and metadata offsets from a 16-byte header.
        void ReadMetadataHeader(const uint8_t *input, uint32_t &encodedWidth, uint32_t &encodedHeight,
                                uint32_t &bitsOffset, uint32_t &refsOffset)
        {
            encodedWidth =
                static_cast<uint32_t>(input[0]) |
                (static_cast<uint32_t>(input[1]) << 8) |
                (static_cast<uint32_t>(input[2]) << 16) |
                (static_cast<uint32_t>(input[3]) << 24);

            encodedHeight =
                static_cast<uint32_t>(input[4]) |
                (static_cast<uint32_t>(input[5]) << 8) |
                (static_cast<uint32_t>(input[6]) << 16) |
                (static_cast<uint32_t>(input[7]) << 24);

            bitsOffset =
                static_cast<uint32_t>(input[8]) |
                (static_cast<uint32_t>(input[9]) << 8) |
                (static_cast<uint32_t>(input[10]) << 16) |
                (static_cast<uint32_t>(input[11]) << 24);

            refsOffset =
                static_cast<uint32_t>(input[12]) |
                (static_cast<uint32_t>(input[13]) << 8) |
                (static_cast<uint32_t>(input[14]) << 16) |
                (static_cast<uint32_t>(input[15]) << 24);
        }

        // Main Decode: decodes an image given width, height and the input stream.
        size_t Decode(uint16_t *output, const int width, const int height, const uint8_t *input, const size_t len)
        {
            uint16_t *outputStart = output;

            uint16_t p0[ENCODING_BLOCK];
            uint16_t p1[ENCODING_BLOCK];
            uint16_t p2[ENCODING_BLOCK];
            uint16_t p3[ENCODING_BLOCK];

            std::vector<uint16_t> bits, refs;
            uint32_t encodedWidth, encodedHeight, bitsOffset, refsOffset;

            ReadMetadataHeader(input, encodedWidth, encodedHeight, bitsOffset, refsOffset);

            if (bitsOffset > len || refsOffset > len)
                return 0;

            if (encodedWidth % ENCODING_BLOCK > 0)
                return 0;

            if (encodedWidth < static_cast<uint32_t>(width))
                return 0;

            // Decode bits and reference metadata
            DecodeMetadata(input, bitsOffset, len, bits);
            DecodeMetadata(input, refsOffset, len, refs);

            size_t offset = METADATA_OFFSET;

            std::vector<uint16_t> row0(encodedWidth);
            std::vector<uint16_t> row1(encodedWidth);
            std::vector<uint16_t> row2(encodedWidth);
            std::vector<uint16_t> row3(encodedWidth);

            int metadataIdx = 0;

            // Process groups of 4 rows:
            for (int y = 0; y < static_cast<int>(encodedHeight); y += 4)
            {
                for (int x = 0; x < static_cast<int>(encodedWidth); x += ENCODING_BLOCK)
                {
                    uint16_t blockBits[4] = {
                        bits[metadataIdx],
                        bits[metadataIdx + 1],
                        bits[metadataIdx + 2],
                        bits[metadataIdx + 3]};
                    uint16_t blockRef[4] = {
                        refs[metadataIdx],
                        refs[metadataIdx + 1],
                        refs[metadataIdx + 2],
                        refs[metadataIdx + 3]};

                    offset += DecodeBlock(&p0[0], blockBits[0], input, offset, len);
                    offset += DecodeBlock(&p1[0], blockBits[1], input, offset, len);
                    offset += DecodeBlock(&p2[0], blockBits[2], input, offset, len);
                    offset += DecodeBlock(&p3[0], blockBits[3], input, offset, len);

                    // Reconstruct rows. The block is assumed to contain two halves; each half provides values.
                    for (int i = 0; i < ENCODING_BLOCK; i += 2)
                    {
                        row0[x + i] = p0[i / 2] + blockRef[0];
                        row0[x + i + 1] = p1[i / 2] + blockRef[1];

                        row1[x + i] = p2[i / 2] + blockRef[2];
                        row1[x + i + 1] = p3[i / 2] + blockRef[3];

                        row2[x + i] = p0[ENCODING_BLOCK / 2 + i / 2] + blockRef[0];
                        row2[x + i + 1] = p1[ENCODING_BLOCK / 2 + i / 2] + blockRef[1];

                        row3[x + i] = p2[ENCODING_BLOCK / 2 + i / 2] + blockRef[2];
                        row3[x + i + 1] = p3[ENCODING_BLOCK / 2 + i / 2] + blockRef[3];
                    }

                    metadataIdx += 4;
                }

                std::memcpy(output, row0.data(), width * sizeof(uint16_t));
                output += width;
                std::memcpy(output, row1.data(), width * sizeof(uint16_t));
                output += width;
                std::memcpy(output, row2.data(), width * sizeof(uint16_t));
                output += width;
                std::memcpy(output, row3.data(), width * sizeof(uint16_t));
                output += width;
            }

            return output - outputStart;
        }

    } // namespace raw
} // namespace motioncam