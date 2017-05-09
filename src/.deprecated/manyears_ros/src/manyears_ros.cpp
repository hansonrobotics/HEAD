/*************************
 *
 * manyears_ros.cpp
 *
 * ROS node to manage the manyears library
 *
 * ***********************/

#include <ros/ros.h>
#include "global_value.h"
#include <manyears_msgs/ManyEarsTrackedAudioSource.h>
#include <rt_audio_ros/AudioStream.h>
//Standard C includes
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <sstream>
#include <string>

#include <sstream>
#include <fstream>

#include "manyears_config.hpp"

using namespace std;

//Manyears includes
extern "C"
{
    //#include "beamformer.h"
    //#include "mixture.h"
#include "overallContext.h"
}

namespace manyears_node{

    class many_ears
    {
    public:
        many_ears(ros::NodeHandle n): local_nh_("~"),
        pub_(n.advertise<manyears_msgs::ManyEarsTrackedAudioSource>("tracked_sources",1000))
        {
            sequence = 0;

            //Init variables
            ros_init_time_ = ros::Time::now();
            frame_number_ = 0;
            //set data to zero
            memset(audio_raw_data_,0,manyears_global::raw_buffer_size_s * sizeof(short));

            //Init ROS connection and parameters
            local_nh_.param("use_audio_stream", use_audio_stream_, false);
            std::string config_file;
            local_nh_.param("config_file", config_file, std::string("../data/rect_cube.mes"));
            std::string raw_file;
            local_nh_.param("raw_file", raw_file, std::string("../data/one_source.raw"));
            local_nh_.param("frame_id", frame_id_, std::string(manyears_global::microphones_frame_s));
            local_nh_.param("instant_time", instant_time_, false);
            local_nh_.param("iterative_path", iterative_path_, std::string(""));
            local_nh_.param("iterative_delay", iterative_delay_, 1);
            local_nh_.param("iterative_enable", iterative_enable_, false);

            local_nh_.param("gain_sep", gain_sep_, 50.0);
            local_nh_.param("gain_pf",  gain_pf_,  50.0);

            if (instant_time_)
                ROS_INFO("Using instantaneous system time for tracked sources.");
            else
                ROS_INFO("Using estimated time from audio stream for tracked sources.");

            //Separation enable
            local_nh_.param("enable_separation", enable_separation_, false);



            //Use raw file if audio stream is not choose
            if(use_audio_stream_ == false){
                ROS_INFO_STREAM("Use "<< raw_file << " ''raw file''\n");
                //Open file in binary mode
                filePtr_ = fopen(raw_file.c_str(), "rb");
                if (filePtr_ == NULL)
                {
                    ROS_ERROR("Invalid file %s \n",raw_file.c_str());
                }

            }
            else
                sub_ = n.subscribe("stream", 100, &many_ears::audio_stream_cb, this);

            //Write into file to sav audio stream
            std::string save_audio_file;
            local_nh_.param("save_audio_file", save_audio_file, std::string(""));
            if(save_audio_file.compare("") != 0)
            {
                is_audio_saved_ = true;
                output_file_ = fopen(save_audio_file.c_str(),"wb");
                if(output_file_ == NULL)
                    ROS_INFO("FILE not open");
            }
            else
                is_audio_saved_ = false;

            libraryContext_ = createEmptyOverallContext();
            //libraryContext_.myParameters = new struct ParametersStruct;

            //Default parameters
            ParametersLoadDefault(libraryContext_.myParameters);
            
            if (iterative_enable_) {
                // Iterative interval delay
                libraryContext_.myParameters->P_OUT_INTERVALDURATION = iterative_delay_ * GLOBAL_FS;
            }
            
            //Change your microphone positions here...
            read_file_for_load_param(config_file);


            // Initialize the microphones
            //microphonesInit(libraryContext_.myMicrophones, manyears_global::nb_microphones_s);
            //Change your microphone positions here...

            //This is a copy of what is in myParameters for now
            setup_microphone_positions_and_gains(libraryContext_.myParameters, libraryContext_.myMicrophones);

            // --- Delete me 
            /*printf("X: %f Y: %f Z: %f\n",libraryContext_.myParameters->P_GEO_MICS_MIC1_X,libraryContext_.myParameters->P_GEO_MICS_MIC1_Y,libraryContext_.myParameters->P_GEO_MICS_MIC1_Z);
            printf("New source TS %f",libraryContext_.myParameters->P_MIXTURE_NEWTHRESHOLD);*/
            //ros::Duration(5).sleep();
            // ---

            // Initialize the preprocessor
            preprocessorInit(libraryContext_.myPreprocessor, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the beamformer
            beamformerInit(libraryContext_.myBeamformer, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the mixture
            mixtureInit(libraryContext_.myMixture, libraryContext_.myParameters);

            // Initialize the gss
            gssInit(libraryContext_.myGSS, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the postfilter
            postfilterInit(libraryContext_.myPostfilter, libraryContext_.myParameters);

            // Initialize the postprocessor
            postprocessorInit(libraryContext_.myPostprocessorSeparated, libraryContext_.myParameters);
            postprocessorInit(libraryContext_.myPostprocessorPostfiltered, libraryContext_.myParameters);

            // Initialize the potential sources
            potentialSourcesInit(libraryContext_.myPotentialSources, libraryContext_.myParameters);

            //Initialize the sources
            trackedSourcesInit(libraryContext_.myTrackedSources, libraryContext_.myParameters);

            // Initialize the separated sources
            separatedSourcesInit(libraryContext_.mySeparatedSources, libraryContext_.myParameters);

            // Initialize the postfiltered sources
            postfilteredSourcesInit(libraryContext_.myPostfilteredSources, libraryContext_.myParameters);

            if (!iterative_enable_)
            {
                // Initialize the output ?
                outputInit(libraryContext_.myOutputSeparated, libraryContext_.myParameters, NULL, NULL, "/tmp/source_sep_****.wav", '*');
                outputInit(libraryContext_.myOutputPostfiltered, libraryContext_.myParameters, NULL, NULL, "/tmp/source_post****.wav", '*');
            }
            else
            {
                // Delete old output
                system(string("exec rm -rf " + iterative_path_ + "*").c_str());

                // Initialize the output
                string sep = iterative_path_ + "source_sep_*****_part$$$$$.wav";
                string post = iterative_path_ + "source_post_*****_part$$$$$.wav";
                outputChunkInit(libraryContext_.myOutputChunkSeparated, libraryContext_.myParameters, NULL, NULL, (char*)sep.c_str(), '*','$');
                outputChunkInit(libraryContext_.myOutputChunkPostfiltered, libraryContext_.myParameters, NULL, NULL, (char*)post.c_str(), '*','$');
            }

        }

        ~many_ears()
        {

            ROS_INFO("~many_ears");



            //terminate
            preprocessorTerminate(libraryContext_.myPreprocessor);
            beamformerTerminate(libraryContext_.myBeamformer);
            mixtureTerminate(libraryContext_.myMixture);
            gssTerminate(libraryContext_.myGSS);
            postfilterTerminate(libraryContext_.myPostfilter);
            postprocessorTerminate(libraryContext_.myPostprocessorSeparated);
            postprocessorTerminate(libraryContext_.myPostprocessorPostfiltered);

            potentialSourcesTerminate(libraryContext_.myPotentialSources);
            trackedSourcesTerminate(libraryContext_.myTrackedSources);
            separatedSourcesTerminate(libraryContext_.mySeparatedSources);
            postfilteredSourcesTerminate(libraryContext_.myPostfilteredSources);

            //Output terminate
            if (!iterative_enable_)
            {
                outputTerminate(libraryContext_.myOutputSeparated);
                outputTerminate(libraryContext_.myOutputPostfiltered);
            }
            else
            {
                outputChunkTerminate(libraryContext_.myOutputChunkSeparated);
                outputChunkTerminate(libraryContext_.myOutputChunkPostfiltered);
            }

            //Close file (will cleanup memory)
            fclose(filePtr_);
            if(is_audio_saved_ == true)
                fclose(output_file_);

        }

        void terminate()
        {
            manyears_msgs::ManyEarsTrackedAudioSource tracked_source_msg;
            //Fill the message header
            //Calculate the estimated time when the trame was catch by the microphones
            
            tracked_source_msg.header.stamp = getTimeStamp();
            //tracked_source_msg.header.stamp = ros_init_time_ + ros::Duration((GLOBAL_FRAMESIZE*GLOBAL_OVERLAP/GLOBAL_FS)*frame_number_);
            tracked_source_msg.header.frame_id = frame_id_;


            pub_.publish(tracked_source_msg);

        }

        ros::Time getTimeStamp()
        {
            if (instant_time_)
                return ros::Time::now();
            else
                return ros_init_time_ + ros::Duration(((float)manyears_global::samples_per_frame_s/manyears_global::sample_rate_s)*frame_number_);
        }

        bool fill_buffer_data_with_raw_file()
        {
            //This does not work well because it does not take account of the channel order
            //bool success = (bool) fread(audio_raw_data_,sizeof(short),manyears_global::raw_buffer_size_s,filePtr_);

            signed short inputShort;
            unsigned int indexBuffer;
            bool success;

            indexBuffer = 0;
            success = true;

            // Load the samples for a complete hop size
            for (int indexSample = 0; indexSample < manyears_global::samples_per_frame_s; indexSample++)
            {
                for (int indexChannel = 0; indexChannel < manyears_global::nb_microphones_s; indexChannel++)
                {
                    if (feof(this->filePtr_)  == 0)
                    {
                        int e = fread(&inputShort, sizeof(short), 1, this->filePtr_);
                        if( e == 0)
                            continue;
                    }
                    else
                    {
                        inputShort = 0;
                        success = false;
                    }
                    this->audio_raw_data_[indexBuffer] = inputShort;
                    indexBuffer++;
                }
            }

            return success;
        }

        void processing_buffer()
        {
            //ROS_INFO("Processing frame : %i \n",frame_number_);
            signed short* buffer_to_write = audio_raw_data_;

            //#1 - Let's create the float data for processing
            for (int channel = 0; channel < manyears_global::nb_microphones_s; channel++)
            {
                for (int frame_index = 0; frame_index < manyears_global::samples_per_frame_s; frame_index++)
                {
                    audio_float_data_[channel][frame_index] = (float) audio_raw_data_[channel + (manyears_global::nb_microphones_s * frame_index)];// / 32768.0;

                    audio_float_data_[channel][frame_index] /= 32768.0;

                    if(is_audio_saved_ == true)
                        fwrite(buffer_to_write++, sizeof(short), 1, output_file_);
                }

                // Copy frames to the beamformer frames, will do 50% overlap internally
                preprocessorPushFrames(libraryContext_.myPreprocessor, manyears_global::samples_per_frame_s, channel);
                preprocessorAddFrame(libraryContext_.myPreprocessor, &audio_float_data_[channel][0], channel, manyears_global::samples_per_frame_s);
            }

            //#2 Preprocess
            preprocessorProcessFrame(libraryContext_.myPreprocessor);

            //#3 Find potential sources from the beamformer
            beamformerFindMaxima(libraryContext_.myBeamformer, libraryContext_.myPreprocessor, libraryContext_.myPotentialSources);

            //#4 Track Sources
            mixtureUpdate(libraryContext_.myMixture, libraryContext_.myTrackedSources, libraryContext_.myPotentialSources);

            if (enable_separation_)
            {

                //#5 Separate sources
                gssProcess(libraryContext_.myGSS, libraryContext_.myPreprocessor, libraryContext_.myTrackedSources, libraryContext_.mySeparatedSources);
                postfilterProcess(libraryContext_.myPostfilter, libraryContext_.mySeparatedSources, libraryContext_.myPreprocessor, libraryContext_.myPostfilteredSources);

                //#6 Postprocess
                postprocessorProcessFrameSeparated(libraryContext_.myPostprocessorSeparated, libraryContext_.myTrackedSources, libraryContext_.mySeparatedSources);
                postprocessorProcessFramePostfiltered(libraryContext_.myPostprocessorPostfiltered, libraryContext_.myTrackedSources, libraryContext_.myPostfilteredSources);
                
                //#7 Output in files
                //outputChunkProcess(libraryContext_.myOutputChunkSeparated, libraryContext_.myPostprocessorSeparated);
                //outputChunkProcess(libraryContext_.myOutputChunkPostfiltered, libraryContext_.myPostprocessorPostfiltered);
                
            }


            //#7 Output result
            //Init ROS msg
            manyears_msgs::ManyEarsTrackedAudioSource tracked_source_msg;
            //Fill the message header
            //Calculate the estimated time when the trame was catch by the microphones
            tracked_source_msg.header.stamp = getTimeStamp();
            //tracked_source_msg.header.stamp = ros_init_time_ + ros::Duration((GLOBAL_FRAMESIZE*GLOBAL_OVERLAP/GLOBAL_FS)*frame_number_);
            tracked_source_msg.header.frame_id = frame_id_;


            //Output to file
            //outputProcess(libraryContext_.myOutputSeparated, libraryContext_.myPostprocessorSeparated);
            //outputProcess(libraryContext_.myOutputPostfiltered, libraryContext_.myPostprocessorPostfiltered);

            

            //Search for tracked source
            for (int source_index = 0; source_index < libraryContext_.myParameters->P_GEN_DYNSOURCES; source_index++)
            {
                if(trackedSourcesGetID(libraryContext_.myTrackedSources,source_index) != -1)
                {
                    /*
					ROS_DEBUG("source index : %i X:%f, Y:%f, Z:%f, id: %i \n",source_index,
						   libraryContext_.myTrackedSources->sourcesPosition[source_index][0],
						   libraryContext_.myTrackedSources->sourcesPosition[source_index][1],
						   libraryContext_.myTrackedSources->sourcesPosition[source_index][2],
						   libraryContext_.myTrackedSources->sourcesID[source_index]);

					*/	

                    //Fill an instance of source info msg
                    manyears_msgs::SourceInfo tracked_source;
                    tracked_source.source_id = trackedSourcesGetID(libraryContext_.myTrackedSources,source_index);
                    tracked_source.source_pos.x = trackedSourcesGetX(libraryContext_.myTrackedSources, source_index);
                    tracked_source.source_pos.y = trackedSourcesGetY(libraryContext_.myTrackedSources, source_index);
                    tracked_source.source_pos.z = trackedSourcesGetZ(libraryContext_.myTrackedSources, source_index);
                    tracked_source.longitude = atan2f(tracked_source.source_pos.y, tracked_source.source_pos.x) * 180/M_PI;
                    tracked_source.latitude = asinf(tracked_source.source_pos.z/
                                                    (tracked_source.source_pos.z*tracked_source.source_pos.z +
                                                     tracked_source.source_pos.y*tracked_source.source_pos.y +
                                                     tracked_source.source_pos.x*tracked_source.source_pos.x )) * 180/M_PI;

/*

                    ROS_INFO("source index : %i X:%f, Y:%f, Z:%f, id: %i \n",source_index,
						   tracked_source.source_pos.x,
						   tracked_source.source_pos.y,
						   tracked_source.source_pos.z,
						   tracked_source.source_id);
*/


                    if (enable_separation_)
                    {

                        int size = (int)((float)GLOBAL_FRAMESIZE * GLOBAL_OVERLAP);


                        //Fill separation data for the source
                        tracked_source.separation_data.resize(size);
                        tracked_source.postfiltered_data.resize(size);


                        postprocessorExtractHop(libraryContext_.myPostprocessorSeparated,
                                                tracked_source.source_id, &tracked_source.separation_data[0]);

                        postprocessorExtractHop(libraryContext_.myPostprocessorPostfiltered,
                                                tracked_source.source_id, &tracked_source.postfiltered_data[0]);

                        //Apply GAIN
                        for (int j = 0; j < size; j++)
                        {
                            tracked_source.separation_data[j] *= gain_sep_;
                            tracked_source.postfiltered_data[j] *= gain_pf_;
                        }

                    }


                    tracked_source.source_probability = 
                        potentialSourcesGetProbability(libraryContext_.myPotentialSources, source_index);
                    //Add it to the tracked sources
                    tracked_source_msg.tracked_sources.push_back(tracked_source);
                }

            }

            // TODO: Figure out why this test is no longer performed, many
            // other modules assume that output ManyEars output should always
            // contain audio data:
            //if(tracked_source_msg.tracked_sources.size() != 0)
            //{
                //Send the message

                tracked_source_msg.sequence = sequence++;
                pub_.publish(tracked_source_msg);
            //}

            // ??????
            //ros::spinOnce();

            //next frame
            frame_number_++;

        }


        bool get_use_audio_stream()
        {
            return use_audio_stream_;
        }

    private:
        void read_file_for_load_param(std::string& file_param_name)
        {
            ROS_INFO("Loading param file %s...",file_param_name.c_str());
            if (!manyears_ros::parseConfigFile(libraryContext_, 
                                               file_param_name)) {
                ROS_ERROR("Could not parse config file.");
            }
        }

        void setup_microphone_positions_and_gains(struct ParametersStruct *parametersStruct, struct objMicrophones* myMicrophones)
        {

            // Set the number of microphones
            microphonesInit(myMicrophones, 8);

            // Add microphone 1...
            microphonesAdd(myMicrophones,
                           0,
                           parametersStruct->P_GEO_MICS_MIC1_X,
                           parametersStruct->P_GEO_MICS_MIC1_Y,
                           parametersStruct->P_GEO_MICS_MIC1_Z,
                           parametersStruct->P_GEO_MICS_MIC1_GAIN
                           );

            // Add microphone 2...
            microphonesAdd(myMicrophones,
                           1,
                           parametersStruct->P_GEO_MICS_MIC2_X,
                           parametersStruct->P_GEO_MICS_MIC2_Y,
                           parametersStruct->P_GEO_MICS_MIC2_Z,
                           parametersStruct->P_GEO_MICS_MIC2_GAIN
                           );

            // Add microphone 3...
            microphonesAdd(myMicrophones,
                           2,
                           parametersStruct->P_GEO_MICS_MIC3_X,
                           parametersStruct->P_GEO_MICS_MIC3_Y,
                           parametersStruct->P_GEO_MICS_MIC3_Z,
                           parametersStruct->P_GEO_MICS_MIC3_GAIN
                           );

            // Add microphone 4...
            microphonesAdd(myMicrophones,
                           3,
                           parametersStruct->P_GEO_MICS_MIC4_X,
                           parametersStruct->P_GEO_MICS_MIC4_Y,
                           parametersStruct->P_GEO_MICS_MIC4_Z,
                           parametersStruct->P_GEO_MICS_MIC4_GAIN
                           );

            // Add microphone 5...
            microphonesAdd(myMicrophones,
                           4,
                           parametersStruct->P_GEO_MICS_MIC5_X,
                           parametersStruct->P_GEO_MICS_MIC5_Y,
                           parametersStruct->P_GEO_MICS_MIC5_Z,
                           parametersStruct->P_GEO_MICS_MIC5_GAIN
                           );

            // Add microphone 6...
            microphonesAdd(myMicrophones,
                           5,
                           parametersStruct->P_GEO_MICS_MIC6_X,
                           parametersStruct->P_GEO_MICS_MIC6_Y,
                           parametersStruct->P_GEO_MICS_MIC6_Z,
                           parametersStruct->P_GEO_MICS_MIC6_GAIN
                           );

            // Add microphone 7...
            microphonesAdd(myMicrophones,
                           6,
                           parametersStruct->P_GEO_MICS_MIC7_X,
                           parametersStruct->P_GEO_MICS_MIC7_Y,
                           parametersStruct->P_GEO_MICS_MIC7_Z,
                           parametersStruct->P_GEO_MICS_MIC7_GAIN
                           );

            // Add microphone 8...
            microphonesAdd(myMicrophones,
                           7,
                           parametersStruct->P_GEO_MICS_MIC8_X,
                           parametersStruct->P_GEO_MICS_MIC8_Y,
                           parametersStruct->P_GEO_MICS_MIC8_Z,
                           parametersStruct->P_GEO_MICS_MIC8_GAIN
                           );


        }

        void audio_stream_cb(const rt_audio_ros::AudioStreamConstPtr& data_in)
        {
            assert(data_in->encoding == rt_audio_ros::AudioStream::SINT_16_PCM);
            assert(data_in->is_bigendian == false);
            assert(data_in->channels == manyears_global::nb_microphones_s);
            assert(data_in->sample_rate == manyears_global::sample_rate_s);
            assert(data_in->data.size() == manyears_global::raw_buffer_size_s * sizeof(int16_t));

            memcpy(
              (uint8_t *) audio_raw_data_,
              &data_in->data[0],
              data_in->data.size());

            processing_buffer();
        }

        //ROS variables
        ros::NodeHandle local_nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        //topic_filters::filtered_publisher pub_;
        ros::Time ros_init_time_;
        bool use_audio_stream_;
        bool is_audio_saved_;
        bool enable_separation_;
        std::string frame_id_;
        bool instant_time_;
        std::string iterative_path_;
        int iterative_delay_;
        bool iterative_enable_;

        // Output gain
        double gain_sep_;
        double gain_pf_;

        //Manyears variables
        int frame_number_;
        short audio_raw_data_[manyears_global::raw_buffer_size_s];
        float audio_float_data_[manyears_global::nb_microphones_s][manyears_global::samples_per_frame_s];
        FILE* filePtr_;
        FILE* output_file_;
        struct objOverall libraryContext_;
        unsigned int sequence;

    };

}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "many_ears");
    ros::NodeHandle n;

    manyears_node::many_ears men(n);

    bool use_callback = men.get_use_audio_stream();

    if(use_callback)
    {
        ROS_INFO("Use stream data");
        ros::spin();
    }
    else
    {
        ROS_INFO("Use file data");
        while(men.fill_buffer_data_with_raw_file() && ros::ok())
        {
            men.processing_buffer();
            ros::spinOnce();
        }


        men.terminate();
        ros::spinOnce();

    }

    return 0;
}

