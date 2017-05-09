#ifndef MANYEARS_NODE_HPP
#define MANYEARS_NODE_HPP

#include <manyears_msgs/ManyEarsTrackedAudioSource.h>
#include <rt_audio_ros/AudioStream.h>
#include <ros/ros.h>

extern "C" {
    #include "overallContext.h"
}

namespace manyears_ros
{
    /// \brief A ManyEars wrapper node.
    ///
    /// Based on the old manyears_ros standalone node, but with more flexible
    /// microphone geometry settings.
    ///
    /// Topics:
    ///  - audio_stream:    Stream input.
    ///  - tracked_sources: Process output.
    ///  - ~stream:         Converted float32 stream for diagnostic purposes.
    ///
    /// Global parameters:
    ///  - frame_id:     The name of the TF frame used for source localization.
    ///                  Default: 'micro_center_link'.
    ///  - microphones:  Microphones geometry definition.
    ///                  Mandatory, see details lower.
    ///  - config_file:  ManyEars config file full path.
    ///                  Note that the microphone geometry parameters from this
    ///                  file are ignored, as this node uses the 'microphones'
    ///                  parameter instead.
    ///                  Mandatory.
    ///  - instant_time: If the stamp header field in the output should be
    ///                  filled with the current time (ros::Time::now()) or an 
    ///                  estimation of the elapsed time based on the number of
    ///                  stream messages received.
    ///                  Default: true.
    ///  - planar_mode:  Only produce poses on a single plane (XY, discards
    ///                  latitude info).
    ///                  Default: false.
    ///  - pre_gain:     Uniform gain applied at input (before sending it to 
    ///                  ManyEars).
    ///                  Default: 1.0.
    ///  - enable_sep:   Enable separation of detected sources.
    ///                  Default: true.
    ///  - gain_sep:     Output gain on separated data.
    ///                  Default: 1.0.
    ///  - gain_pf:      Output gain on postfiltered data.
    ///                  Default: 1.0.
    ///
    /// Parameters for microphone geometry
    /// ----------------------------------
    /// This node looks for an array of microphone descriptions in the
    /// 'microphones' sub-namespace.
    /// Each element has two members:
    ///  - 'pos', a struct of cartesian coordinates ({x: X, y: Y, z: Z}).
    ///  - 'gain', a float for the microphone gain.
    /// A complete microphone description for a stereo could look like this:
    ///   manyears_node:
    ///     microphones:
    ///       - {pos: {x: 0.00, y:  0.05, z: 0.00}, gain: 1.0}
    ///       - {pos: {x: 0.00, y: -0.05, z: 0.00}, gain: 1.0}
    ///  
    /// Note that at least two microphones have to be defined.
    ///
    class ManyEarsNode
    {
    private:

        ros::Subscriber sub_audio_;
        ros::Publisher  pub_sources_;
        ros::Publisher  pub_stream_;

        /// \brief Internal microphone definition structure used when parsing
        /// parameters.
        struct MicDef
        {
            double gain;
            double pos[3];

            MicDef(): 
                gain(1.0)
            {
                std::fill(&pos[0], &pos[3], 0.0);
            }
        };
        std::vector<MicDef> mic_defs_;

        std::string frame_id_;
        bool        instant_time_;
        bool        planar_mode_;
        float       pre_gain_;
        bool        enable_sep_;
        float       gain_sep_;
        float       gain_pf_;

        ros::Time processed_time_;   // Estimated processed time from
                                     // initialization.
        int       processed_frames_; // Actual number of processed frames.

        typedef std::vector<float> FloatVec;     // Internal flat buffer.
        FloatVec                   buffer_flat_; // Will contain a flat 
                                                 // ManyEars-sized audio buffer
                                                 // ready to be splitted and
                                                 // sent to the library.


        struct objOverall manyears_context_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        ManyEarsNode(ros::NodeHandle& n, ros::NodeHandle& np);

        ~ManyEarsNode();

    private:
        int       microphonesCount() const { return mic_defs_.size(); }
        ros::Time getTimeStamp()     const;

        bool parseParams(const ros::NodeHandle& np);
        bool parseConfigFile(const std::string& fn);
        void initPipeline();

        void audioCB(const rt_audio_ros::AudioStream::ConstPtr& msg);
        void process();
    };
        
}

#endif

