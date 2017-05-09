/// \file stream_extract.cpp A simple tool that writes raw audio files from a
///                          rt_audio_ros stream.
///                          NOTE: Disabled VAD input support.
#include <ros/ros.h>
#include <rt_audio_ros/AudioStream.h>
#include <manyears_msgs/ManyEarsTrackedAudioSource.h>
//#include <audio_tools/VADStream.h>
#include <iostream>
#include <limits>

namespace {

    bool use_pf_ = false;

    void writeByteStream(const std::vector<unsigned char>& data)
    {
        std::cout.write((char*)&(data[0]), data.size());
    }

    void audioCB(const rt_audio_ros::AudioStream::ConstPtr& msg)
    {
        writeByteStream(msg->data);
    }

    void manyearsCB(const manyears_msgs::ManyEarsTrackedAudioSource& msg)
    {
        if (msg.tracked_sources.size() < 1) {
            return;
        }
        // Always extract the first source only:
        const manyears_msgs::SourceInfo& source = msg.tracked_sources[0];
        const std::vector<float>& data = use_pf_ ? source.postfiltered_data
                                                 : source.separation_data;
        std::vector<float>::const_iterator i = data.begin();
        while (i != data.end()) {
            float s = *(i++);
            std::cout.write((const char *)(&s), sizeof(float));
        }

    }

    //void voiceCB(const audio_tools::VADStream::ConstPtr& msg)
    //{
    //    if (msg->is_VAD) {
    //        writeByteStream(msg->data);
    //    }
    //}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stream_extract");

    ros::NodeHandle n;
    ros::Subscriber sub;

    if (argc > 1) {
        if (strncmp(argv[1], "-m", 2) == 0) {
            if (argv[1][2] == 'p') {
                use_pf_ = true;
            }
            sub = n.subscribe("tracked_sources", 10, manyearsCB);
        } 
        //else if (strcmp(argv[1], "-v") == 0) {
        //    sub = n.subscribe("voice_stream", 10, voiceCB);
        //}
    } else {
        sub = n.subscribe("audio_stream", 10, audioCB);
    }

    ros::spin();
}

