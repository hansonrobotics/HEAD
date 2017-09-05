Header header

uint8 encoding

uint8 SINT_8_PCM=0
uint8 UINT_8_PCM=1
uint8 SINT_16_PCM=2
uint8 SINT_24_PCM=3
uint8 SINT_32_PCM=4
uint8 FLOAT_32=5
uint8 FLOAT_64=6

uint8  is_bigendian
uint8  channels
uint32 sample_rate

# interleaved channels
uint8[]  data
