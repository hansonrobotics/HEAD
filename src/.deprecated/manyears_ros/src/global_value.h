#ifndef GLOBAL_VALUE_H_
#define GLOBAL_VALUE_H_

extern "C"{
#include <parameters.h>
}
#include <string>

namespace manyears_global
{

	static const int sample_rate_s = GLOBAL_FS; //48000 samples/sec
	static const int samples_per_frame_s = GLOBAL_FRAMESIZE * GLOBAL_OVERLAP; //512
	static const int nb_microphones_s = GLOBAL_MICSNUMBER; //8
	static const unsigned int raw_buffer_size_s = samples_per_frame_s * nb_microphones_s;
	static const std::string microphones_frame_s = "micro_center_link";

}

#endif
