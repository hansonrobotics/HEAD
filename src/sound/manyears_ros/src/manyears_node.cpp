#include <manyears_ros/manyears_node.hpp>
#include "manyears_config.hpp"
#include "global_value.h" // ManyEars static consts.
#include <inttypes.h>

using namespace manyears_ros;

namespace {

    /// \brief Replacement function for beamformer initialization that uses an
    /// half-circle arc instead of a full sphere.
    void beamformerArcInit(struct objBeamformer*    myBeamformer,
                           struct ParametersStruct* myParameters, 
                           struct objMicrophones*   myMicrophones)
    {
        // Copied directly from ManyEars lib source with a few formatting
        // changes to fit the current source file.

        myBeamformer->BF_SPHERENBLEVELS = GLOBAL_SPHERE_NUMBERLEVELS;
        myBeamformer->BF_MAXSOURCES     = myParameters->P_BF_MAXSOURCES;
        myBeamformer->BF_FILTERRANGE    = myParameters->P_BF_FILTERRANGE;
        myBeamformer->BF_RESETRANGE     = myParameters->P_BF_RESETRANGE;
        myBeamformer->BF_ET             = myParameters->P_BF_ET;

        myBeamformer->myMicrophones = 
            (struct objMicrophones*) malloc(sizeof(struct objMicrophones));
        microphonesClone(myMicrophones, myBeamformer->myMicrophones);

        myBeamformer->mySphere = 
            (struct objSphere*) malloc(sizeof(struct objSphere));
        sphereArcInit(myBeamformer->mySphere, 
                      -90,
                      90,
                      100);

        myBeamformer->myDelays = 
            (struct objDelays*) malloc(sizeof(struct objDelays));
        delaysInit(myBeamformer->myDelays,
                   myBeamformer->myMicrophones,
                   myBeamformer->mySphere, 
                   GLOBAL_C, 
                   GLOBAL_FS, 
                   1.5);

        myBeamformer->myRij = (struct objRij*) malloc(sizeof(struct objRij));
        rijInit(myBeamformer->myRij,
                myParameters,
                myBeamformer->myMicrophones,
                myBeamformer->myDelays,
                GLOBAL_FRAMESIZE,
                myParameters->P_BF_FILTERRANGE,
                myParameters->P_BF_RESETRANGE);

        myBeamformer->maxValues = 
            (float*) newTable1D(myBeamformer->BF_MAXSOURCES, sizeof(float));
        myBeamformer->maxIndexes =
            (signed int*) newTable1D(myBeamformer->BF_MAXSOURCES, 
                                     sizeof(signed int));

    }
}

ManyEarsNode::ManyEarsNode(ros::NodeHandle& n, ros::NodeHandle& np):
    processed_frames_(0)
{
    manyears_context_ = createEmptyOverallContext();
    ParametersLoadDefault(manyears_context_.myParameters);

    if (!parseParams(np)) {
        ROS_ERROR("Could not parse ManyEars parameters correctly, the node "
                  "will not be initialized.");
        return;
    }

    ROS_INFO("Initializing ManyEars with %i microphones...",
             microphonesCount());

    initPipeline();

    sub_audio_ = n.subscribe("audio_stream", 10, &ManyEarsNode::audioCB, this);

    pub_sources_ = n.advertise<manyears_msgs::ManyEarsTrackedAudioSource>(
        "tracked_sources",
        10);

    pub_stream_  = np.advertise<rt_audio_ros::AudioStream>("stream", 100);

}

ManyEarsNode::~ManyEarsNode()
{
    preprocessorTerminate(       manyears_context_.myPreprocessor);
    beamformerTerminate(         manyears_context_.myBeamformer);
    mixtureTerminate(            manyears_context_.myMixture);
    gssTerminate(                manyears_context_.myGSS);
    postfilterTerminate(         manyears_context_.myPostfilter);
    postprocessorTerminate(      manyears_context_.myPostprocessorSeparated);
    postprocessorTerminate(      manyears_context_.myPostprocessorPostfiltered);
    potentialSourcesTerminate(   manyears_context_.myPotentialSources);
    trackedSourcesTerminate(     manyears_context_.myTrackedSources);
    separatedSourcesTerminate(   manyears_context_.mySeparatedSources);
    postfilteredSourcesTerminate(manyears_context_.myPostfilteredSources);
}

bool ManyEarsNode::parseParams(const ros::NodeHandle& np)
{
    std::string config_fn;
    np.param("config_file", config_fn, std::string(""));
    if (config_fn == std::string("")) {
        ROS_INFO("No config_file defined, will use default parameters.");
    } else {
        std::string config_fn;
        np.getParam("config_file", config_fn);
        if (!manyears_ros::parseConfigFile(manyears_context_, config_fn)) {
            return false;
        }
    }

    if (!np.hasParam("microphones")) {
        ROS_ERROR("No 'microphones' parameter - cannot define array geometry.");
        return false;
    }

    using namespace XmlRpc;
    XmlRpcValue mic_n;
    np.getParam("microphones", mic_n);
    if (mic_n.getType() != XmlRpcValue::TypeArray) {
        ROS_ERROR("'microphones' is not an array.");
        return false;
    }

    for (int i = 0; i < mic_n.size(); ++i) {
        MicDef md;

        XmlRpcValue& v = mic_n[i];
        if (v.getType() != XmlRpcValue::TypeStruct) {
            ROS_ERROR("'microphones' element %i is not a struct, skipping",
                      i);
            continue;
        }

        if (!v.hasMember("gain")) {
            ROS_ERROR("No gain specified for microphone %i, using '1.0'",
                      i);
        } else {
            XmlRpcValue& gv = v["gain"];
            if (gv.getType() != XmlRpcValue::TypeDouble) {
                ROS_ERROR("Gain element of microphone %i is not a double, "
                          "using '1.0'",
                          i);
            } else {
                md.gain = gv;
            }
        }

        if (!v.hasMember("pos")) {
            ROS_ERROR("No pos specified for microphone %i, using '(0, 0, 0)'",
                      i);
        } else {
            XmlRpcValue& posv = v["pos"];
            if (posv.getType() != XmlRpcValue::TypeStruct) {
                ROS_ERROR("Pos element of microphone %i is not a struct, "
                          "using (0, 0, 0).");
            } else {
                if (posv.hasMember("x")) {
                    XmlRpcValue& xv = posv["x"];
                    if (xv.getType() == XmlRpcValue::TypeDouble) {
                        md.pos[0] = xv;
                    } else {
                        ROS_WARN("'x' element of microphone %i is not a "
                                 "double, using 0.0.",
                                 i);
                    }
                }
                if (posv.hasMember("y")) {
                    XmlRpcValue& yv = posv["y"];
                    if (yv.getType() == XmlRpcValue::TypeDouble) {
                        md.pos[1] = yv;
                    } else {
                        ROS_WARN("'y' element of microphone %i is not a "
                                 "double, using 0.0.",
                                 i);
                    }
                }
                if (posv.hasMember("z")) {
                    XmlRpcValue& zv = posv["z"];
                    if (zv.getType() == XmlRpcValue::TypeDouble) {
                        md.pos[2] = zv;
                    } else {
                        ROS_WARN("'z' element of microphone %i is not a "
                                 "double, using 0.0.",
                                 i);
                    }
                }
            }
        }

        mic_defs_.push_back(md);
    }

    if (microphonesCount() < 2) {
        ROS_ERROR("Less than two microphones were defined.");
        return false;
    }

    np.param("frame_id",     frame_id_,     std::string("micro_center_link"));
    np.param("instant_time", instant_time_,                             true);
    np.param("planar_mode",  planar_mode_,                             false);
    double pg;
    np.param("pre_gain",     pg,                                         1.0);
    pre_gain_ = pg;

    double gain_sep, gain_pf;
    np.param("gain_sep", gain_sep, 1.0);
    np.param("gain_pf", gain_pf, 1.0);
    gain_sep_ = gain_sep;
    gain_pf_  = gain_pf;

    return true;
}

void ManyEarsNode::initPipeline()
{
    microphonesInit(manyears_context_.myMicrophones, mic_defs_.size());
    for (int i = 0; i < mic_defs_.size(); ++i) {
        const MicDef& md = mic_defs_[i];
        microphonesAdd(manyears_context_.myMicrophones,
                      i,
                      md.pos[0],
                      md.pos[1],
                      md.pos[2],
                      md.gain);
    }

    preprocessorInit(       manyears_context_.myPreprocessor,
                            manyears_context_.myParameters,
                            manyears_context_.myMicrophones);
    if (planar_mode_) {
        // In planar mode, we replace the default sphere by an arc of 180
        // degrees on the XY plane.
        // This is done in the replacement beamformerArcInit function.
        // TODO: Parameters for angles, number of points.
        beamformerArcInit(  manyears_context_.myBeamformer,
                            manyears_context_.myParameters,
                            manyears_context_.myMicrophones);
    } else {
        beamformerInit(     manyears_context_.myBeamformer,
                            manyears_context_.myParameters,
                            manyears_context_.myMicrophones);
    }
    mixtureInit(            manyears_context_.myMixture,
                            manyears_context_.myParameters);
    gssInit(                manyears_context_.myGSS,
                            manyears_context_.myParameters,
                            manyears_context_.myMicrophones);
    postfilterInit(         manyears_context_.myPostfilter,
                            manyears_context_.myParameters);
    postprocessorInit(      manyears_context_.myPostprocessorSeparated,
                            manyears_context_.myParameters);
    postprocessorInit(      manyears_context_.myPostprocessorPostfiltered,
                            manyears_context_.myParameters);
    potentialSourcesInit(   manyears_context_.myPotentialSources,
                            manyears_context_.myParameters);
    trackedSourcesInit(     manyears_context_.myTrackedSources,
                            manyears_context_.myParameters);
    separatedSourcesInit(   manyears_context_.mySeparatedSources,
                            manyears_context_.myParameters);
    postfilteredSourcesInit(manyears_context_.myPostfilteredSources,
                            manyears_context_.myParameters);

}

ros::Time ManyEarsNode::getTimeStamp() const
{
    if (instant_time_) {
        return ros::Time::now();
    } else {
        return processed_time_;
    }
}

void ManyEarsNode::audioCB(const rt_audio_ros::AudioStream::ConstPtr& msg)
{
    // Fixed ManyEars input buffer size, in samples:
    const int total_samples_count = manyears_global::samples_per_frame_s *
                                    microphonesCount();

    // NOTE: Always calculate processed time, even if frames are skipped:
    if (processed_time_.isZero()) {
        processed_time_ = ros::Time::now();
    } else {
        processed_time_ += ros::Duration(
            float(manyears_global::samples_per_frame_s) / 
            float(manyears_global::sample_rate_s));
    }
   
    if (msg->channels != microphonesCount()) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet with %i channels, "
                           "expected %i. Packet skipped.",
                            msg->channels,
                            microphonesCount());
        return;
    }

    if (msg->encoding != rt_audio_ros::AudioStream::SINT_16_PCM) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet not in "
                           "SINT_16_PCM format. Packet skipped.");
        return;
    }

    if (msg->is_bigendian != false) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet not in "
                           "little endian. Packet skipped.");
        return;
    }

    if (msg->sample_rate != manyears_global::sample_rate_s) {
        ROS_ERROR_THROTTLE(1.0,
                           "Received an audio stream packet sampled at "
                           "%i Hz, expecting %i. Packet skipped.",
                           msg->sample_rate,
                           manyears_global::sample_rate_s);
        return;
    }
    
    // First, do a straight short -> float conversion.
    const int      buffer_in_size = msg->data.size() / sizeof(int16_t);
    const int16_t* buffer_in      = 
        reinterpret_cast<const int16_t*>(&(msg->data[0]));

    int i = 0; // Input index, incremented at reading.
    while (i < buffer_in_size) {
        float v = pre_gain_ * float(buffer_in[i++]) / SHRT_MAX;
        buffer_flat_.push_back(v);
    }
    process();
    buffer_flat_.clear(); // Cleared for next round.
}

void ManyEarsNode::process()
{
    // TODO: Allocation should be done at init.
    typedef std::vector<FloatVec> MESplitBuffer; // ManyEars input buffer.
    MESplitBuffer buffer = MESplitBuffer(microphonesCount());
    for (int i = 0; i < buffer.size(); ++i) {
        buffer[i].resize(manyears_global::samples_per_frame_s);
    }
        
    // Split in separate channels for ManyEars.
    int i = 0; // Incremented when buffer_flat_ read.
    for (int s = 0; s < manyears_global::samples_per_frame_s; ++s) {
        for (int c = 0; c < microphonesCount(); ++c) {
            buffer[c][s] = buffer_flat_[i++];
        }
    }


    // Push all frames to ManyEars and process.
    for (int i = 0; i < buffer.size(); ++i) {
        preprocessorPushFrames(manyears_context_.myPreprocessor,
                               buffer[i].size(),
                               i);
        preprocessorAddFrame(  manyears_context_.myPreprocessor,
                               &(buffer[i][0]),
                               i,
                               buffer[i].size());
    }
    preprocessorProcessFrame(manyears_context_.myPreprocessor);
    beamformerFindMaxima(    manyears_context_.myBeamformer,
                             manyears_context_.myPreprocessor,
                             manyears_context_.myPotentialSources);
    mixtureUpdate(           manyears_context_.myMixture,
                             manyears_context_.myTrackedSources,
                             manyears_context_.myPotentialSources);

    if (enable_sep_) {
        gssProcess(       manyears_context_.myGSS,
                          manyears_context_.myPreprocessor,
                          manyears_context_.myTrackedSources,
                          manyears_context_.mySeparatedSources);
        postfilterProcess(manyears_context_.myPostfilter,
                          manyears_context_.mySeparatedSources,
                          manyears_context_.myPreprocessor,
                          manyears_context_.myPostfilteredSources);
        postprocessorProcessFrameSeparated(
            manyears_context_.myPostprocessorSeparated,
            manyears_context_.myTrackedSources,
            manyears_context_.mySeparatedSources);
        postprocessorProcessFramePostfiltered(
            manyears_context_.myPostprocessorPostfiltered,
            manyears_context_.myTrackedSources,
            manyears_context_.myPostfilteredSources);
    }

    // Debug output.
    const objBeamformer*       bf    = manyears_context_.myBeamformer;
    const objPotentialSources* psrcs = manyears_context_.myPotentialSources;
    ROS_DEBUG_THROTTLE(0.2,
                       "P_0: %f",
                       psrcs->sourcesProbability[0]);

    // Output formatting.
    manyears_msgs::ManyEarsTrackedAudioSource msg_out;
    msg_out.header.frame_id = frame_id_;
    msg_out.header.stamp    = getTimeStamp();
    msg_out.sequence        = processed_frames_++;

    objTrackedSources* sources = manyears_context_.myTrackedSources;
    for (int i = 0; i < manyears_context_.myParameters->P_GEN_DYNSOURCES; ++i) {
        int src_id = trackedSourcesGetID(sources, i);
        if (src_id != -1) {
            int last_s = msg_out.tracked_sources.size();
            msg_out.tracked_sources.resize(last_s + 1);
            manyears_msgs::SourceInfo& src = msg_out.tracked_sources[last_s];

            src.source_id = src_id; 

            double& px = src.source_pos.x;
            double& py = src.source_pos.y;
            double& pz = src.source_pos.z;
            px = trackedSourcesGetX(sources, i);
            py = trackedSourcesGetY(sources, i);
            pz = trackedSourcesGetZ(sources, i);

            double long_rad = atan2(py, px);
            double lat_rad  = asin( pz / (px*px + py*py + pz*pz));

            src.longitude = long_rad * 180.0 / M_PI;
            src.latitude  = lat_rad  * 180.0 / M_PI;

            if (planar_mode_) {
                // Re-generate postion based on longitude only:
                px = cos(long_rad);
                py = sin(long_rad);
                pz = 0.0;
                src.latitude = 0.0;
            }

            if (enable_sep_) {
                static const int SIZE = GLOBAL_FRAMESIZE * GLOBAL_OVERLAP;

                src.separation_data.resize(SIZE);
                postprocessorExtractHop(
                    manyears_context_.myPostprocessorSeparated,
                    src.source_id,
                    &(src.separation_data[0]));
                src.postfiltered_data.resize(SIZE);
                postprocessorExtractHop(
                    manyears_context_.myPostprocessorPostfiltered,
                    src.source_id,
                    &(src.postfiltered_data[0]));

                // Output gain.
                for (int j = 0; j < SIZE; ++j) {
                    src.separation_data[j]   *= gain_sep_;
                    src.postfiltered_data[j] *= gain_pf_;
                }
            }

            src.source_probability = potentialSourcesGetProbability(
                manyears_context_.myPotentialSources,
                i);
        }
    }

    pub_sources_.publish(msg_out);

    if (pub_stream_.getNumSubscribers() > 0) {
        rt_audio_ros::AudioStream flat_stream;
        flat_stream.header      = msg_out.header;
        flat_stream.encoding    = rt_audio_ros::AudioStream::FLOAT_32;
        flat_stream.channels    = microphonesCount();
        flat_stream.sample_rate = manyears_global::sample_rate_s;

        flat_stream.data.resize(buffer_flat_.size() * sizeof(float));
        std::copy(buffer_flat_.begin(),
                  buffer_flat_.end(),
                  (float*)(&(flat_stream.data[0])));

        pub_stream_.publish(flat_stream);
    }
}

