#include <rt_audio_ros/AudioStream.h>
#include <ros/ros.h>

namespace
{
    ros::Publisher pub_audio_;
    int            rate_      = 1;

    void pushMsg(rt_audio_ros::AudioStream& out)
    {
        pub_audio_.publish(out);
        out.data.clear();
    }

    void audioCB(const rt_audio_ros::AudioStream& msg)
    {
        rt_audio_ros::AudioStream out;

        out.header       = msg.header;
        out.encoding     = msg.encoding;
        out.is_bigendian = msg.is_bigendian;
        out.channels     = msg.channels;
        out.sample_rate  = rate_ * msg.sample_rate;

        int ss; // Sample size, in bytes.
        switch (msg.encoding) {
            case rt_audio_ros::AudioStream::SINT_8_PCM:
            case rt_audio_ros::AudioStream::UINT_8_PCM:
                ss = 1;
                break;
            case rt_audio_ros::AudioStream::SINT_16_PCM:
                ss = 2;
                break;
            case rt_audio_ros::AudioStream::SINT_24_PCM:
                ss = 3;
                break;
            case rt_audio_ros::AudioStream::FLOAT_32:
                ss = 4;
                break;
            case rt_audio_ros::AudioStream::FLOAT_64:
                ss = 8;
                break;
            default:
                ROS_ERROR("Unknown encoding.");
                return;
        }

        int i = 0;
        const unsigned char* in     = &(msg.data[0]);
        const int            nc     = msg.channels; 
        const int            s_size = ss * msg.channels;

        out.data.reserve(msg.data.size());

        while (i < msg.data.size()) {
            for (int j = 0; j < s_size; ++j) {
                out.data.push_back(in[i++]);
            }
            if (out.data.size() == msg.data.size()) {
                pushMsg(out);
            }
            for (int j = 0; j < (rate_ - 1); ++j) {
                for (int k = 0; k < s_size; ++k) {
                    out.data.push_back(0);
                }
                if (out.data.size() == msg.data.size()) {
                    pushMsg(out);
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "upscaler");

    ros::NodeHandle n, np("~");

    np.param("rate", rate_, 1);

    ros::Subscriber sub_audio = n.subscribe("audio_stream", 10, &::audioCB);

    pub_audio_ = n.advertise<rt_audio_ros::AudioStream>("audio_stream_up", 10);

    ros::spin();

    return 0;
}

