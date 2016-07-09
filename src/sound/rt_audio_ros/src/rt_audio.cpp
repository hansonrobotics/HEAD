#include "RTAudio/RtAudio.h"
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

//ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rt_audio_ros/AudioStream.h>
//#include "topic_filters/filtered_publisher.hpp"


namespace rt_audio_ros
{
	//int record( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
	//		 double streamTime, RtAudioStreamStatus status, void *userData );

	class rt_audio
	{
	public:

		rt_audio(ros::NodeHandle n): local_nh_("~"),
			pub_( n.advertise<AudioStream>("audio_stream",100) ),
			frame_number_(0),
            now_(ros::Time())
		{
			// Advertise ROS msg of audio stream
			local_nh_.param("save_raw_file",save_raw_file_, false);

            // Microphones count.
            int nb_mic;
            local_nh_.param("nb_microphones", nb_mic, 8);
            nb_microphones_ = nb_mic; // Signed/unsigned conversion.

	    // Sample rate.
            int sr;
            local_nh_.param("sample_rate", sr, 48000);
            sample_rate_ = sr; // Signed/unsigned conversion.
	   
	    // Samples per frame.
            int spf;
            local_nh_.param("samples_per_frame", spf, 512);
            samples_per_frame_ = spf; // Signed/unsigned conversion.
            
	    // Card to use (examples : "USB" or "Intel").
	    local_nh_.param("card_name", card_name_hint_, std::string(""));

			//Init
			rtaudio_ = new RtAudio();
			if ( rtaudio_->getDeviceCount() < 1 ) {
				ROS_ERROR("No audio devices found!");
			}
			// Determine the number of devices available
			unsigned int numberDevices = rtaudio_->getDeviceCount();

			// Variable to stock the approriate device
			uint selected_device = -1;

			// Scan through devices for various capabilities
			RtAudio::DeviceInfo info;

			for (unsigned int indexDevice = 0; indexDevice < numberDevices; indexDevice++ )
			{
				info = rtaudio_->getDeviceInfo(indexDevice);
                if ( info.probed == true )
                {
                    ROS_INFO_STREAM( "Card " << indexDevice << " " << info.name.c_str() << 
                            ". Inputs : " << info.inputChannels );

                    ROS_INFO_STREAM("sp rates: ");
                    for (int i = 0; i < info.sampleRates.size(); ++i) {
                        ROS_INFO_STREAM(info.sampleRates[i] << " ");
                    }
                    // Take the device with enough channels AND with the good name hint
                    if(( info.inputChannels >= nb_microphones_)
                            && ( std::count(info.sampleRates.begin(), // If sample is supported
                                    info.sampleRates.end(), sample_rate_) > 0 )
                            && boost::contains( info.name, card_name_hint_ ) )
                    {
                        selected_device = indexDevice;
                        break;
                    }
                }
            }

			RtAudio::StreamParameters iparameters;
			//Take the last device for the input (normally the PCI sound card)
			iparameters.deviceId = selected_device;
			iparameters.nChannels = nb_microphones_;
			iparameters.firstChannel = 0;
			RtAudio::StreamOptions audioOptions;
			int sampleRate = sample_rate_;
			unsigned int bufferFrames = samples_per_frame_; // Hop size.

			//Write into file to sav audio stream
			if(save_raw_file_)
			{
				//Get now date
				time_t rawtime;
				struct tm * timeinfo;
				char buffer[80];
				time(&rawtime);
				timeinfo = localtime(&rawtime);
				strftime(buffer, 80, "rtaudio_%Y-%m-%d_%H-%M-%S.raw", timeinfo);

				std::string save_file_path = std::string("./") + buffer;
				//output_file_ = fopen("data/test_capture_audio.raw","wb");
				output_file_ = fopen(save_file_path.c_str(),"wb");
				if(output_file_ == NULL)
					ROS_INFO("FILE %s not open", save_file_path.c_str());
				else
					ROS_INFO("Save FILE");
			}
			try {
				printf("deviceId : %d, nbChannels : %d, sample rate : %d, bufferSize : %d"
					, iparameters.deviceId, iparameters.nChannels, sampleRate, bufferFrames);

				rtaudio_->openStream( NULL, &iparameters, RTAUDIO_SINT16, sampleRate, &bufferFrames, &rt_audio::record, this, &audioOptions);
				ROS_INFO("Open done");
				rtaudio_->startStream();
			}
			catch ( RtError& e ) {
				e.printMessage();
				exit( 0 );
			}

            // The now_ field will be updated on the first frame's reception.
            frame_duration_ = ros::Duration(float(spf) / float(sr));

			ROS_INFO("End of RtAudio init");

		}

		~rt_audio()
		{
			//destruction
			try {
				// Stop the stream
				rtaudio_->stopStream();
			}
			catch (RtError& e) {
				e.printMessage();
			}

			if ( rtaudio_->isStreamOpen() ) rtaudio_->closeStream();
			delete rtaudio_;
			//close file
			if(save_raw_file_)
				fclose(output_file_);
		}

		void write_in_file(const char* data, size_t size)
		{
			fwrite(data, sizeof(char), size, output_file_);
		}

		bool get_save_raw_file()
		{
			return save_raw_file_;
		}

	private:
        static int record( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
                 double streamTime, RtAudioStreamStatus status, void *userData )
        {
            if ( status )
                std::cout << "Stream overflow detected!" << std::endl;

            rt_audio_ros::rt_audio* rtaudio_ptr = static_cast<rt_audio_ros::rt_audio*>(userData);

            size_t size = nBufferFrames * rtaudio_ptr->nb_microphones_ * sizeof(int16_t);

            // We lock time stamp production to the number of frames received:
            if (rtaudio_ptr->now_.isZero())
                rtaudio_ptr->now_ = ros::Time::now();
            rtaudio_ptr->now_ += rtaudio_ptr->frame_duration_;

            AudioStream audio_stream_msg;
            audio_stream_msg.header.stamp = rtaudio_ptr->now_;
            audio_stream_msg.encoding = AudioStream::SINT_16_PCM;
            audio_stream_msg.is_bigendian = false;
            audio_stream_msg.channels = rtaudio_ptr->nb_microphones_;
            audio_stream_msg.sample_rate = rtaudio_ptr->sample_rate_;
            audio_stream_msg.data.assign(
                (uint8_t *) inputBuffer,
                (uint8_t *) inputBuffer + ( size )); 

            rtaudio_ptr->pub_.publish(audio_stream_msg);

            if (rtaudio_ptr->save_raw_file_)
                rtaudio_ptr->write_in_file((const char*)inputBuffer, size);
            return 0;
        }

		//ROS variables
		ros::NodeHandle local_nh_;
		ros::Publisher pub_;
		//topic_filters::filtered_publisher pub_;
		//RT audio variables
		RtAudio* rtaudio_;
		uint frame_number_;
		
		bool save_raw_file_;
		FILE* output_file_;
        	
		unsigned int nb_microphones_;
        	unsigned int sample_rate_;
        	unsigned int samples_per_frame_;
		std::string card_name_hint_;

        ros::Time now_;
        ros::Duration frame_duration_;
	};


}

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "rt_audio_publisher");
	ros::NodeHandle n;	

	rt_audio_ros::rt_audio ra(n);

	//ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	//spinner.spin();
	ros::spin();

	return 0;
}

