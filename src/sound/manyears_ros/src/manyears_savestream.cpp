#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <limits.h>

//ROS includes
#include <ros/ros.h>
//#include <ros/package.h>
//#include "global_value.h"
#include <manyears_msgs/ManyEarsTrackedAudioSource.h>

//BY DL
#include <sstream>
#include <fstream>

using namespace std;

//Manyears includes
extern "C"
{
//    #include "overallContext.h"
}

namespace manyears_savestream_node
{

    class manyears_savestream
    {
    public:

        manyears_savestream()
        {
            //The topic we are listening to
            nh_.param("topic",topic_,std::string("tracked_sources"));
            sub_ = nh_.subscribe(topic_,1000, &manyears_savestream::audio_source_cb, this);

        }

        ~manyears_savestream()
        {

        }

        void save_wave(int id)
        {

            using namespace std;

            if (streamData_.find(id) != streamData_.end())
            {
                ROS_INFO("save_wave(int id = %i)",id);

                std::vector<float> &vect = streamData_[id];

                unsigned short nChannels = 1;
                unsigned int sizeSample = sizeof(short);//Each sample will be converted to short
                unsigned int subChunk1Size = 16; //PCM
                unsigned int subChunk2Size = nChannels * vect.size() * sizeSample;
                unsigned int sampleRate = 48000;
                unsigned short audioFormat = 1; //PCM
                unsigned int chunkSize = 4 + (8 + subChunk1Size) + (8 + subChunk2Size);
                unsigned int byteRate = sampleRate * nChannels * sizeSample;
                unsigned short blockAlign = nChannels * sizeSample;
                unsigned short bitsPerSample =  8 * sizeSample;

                //OPEN FILE
                ostringstream temp;
                temp << id;
                ofstream outFile((string("/tmp/source_") + temp.str() + string(".wav")).c_str(),ios::binary);


                //WAV HEADER
                char riff_start[4] = {'R','I','F','F'};
                outFile.write(riff_start,4);

                outFile.write((char*)&chunkSize,sizeof(chunkSize));

                char wave_start[4] = {'W','A','V','E'};
                outFile.write(wave_start,4);


                char fmt_start[4] = {'f','m','t',' '};
                outFile.write(fmt_start,4);


                outFile.write((char*)&subChunk1Size, sizeof(subChunk1Size)); //should have 16 bytes of fmt

                outFile.write((char*)&audioFormat, sizeof(audioFormat)); //2

                outFile.write((char*)&nChannels, sizeof(nChannels)); //2
                outFile.write((char*)&sampleRate,sizeof(sampleRate)); //4
                outFile.write((char*)&byteRate,sizeof(byteRate));//4
                outFile.write((char*)&blockAlign,sizeof(blockAlign));//2
                outFile.write((char*)&bitsPerSample,sizeof(bitsPerSample));//2
                char data_start[4] = {'d','a','t','a'};
                outFile.write(data_start,4);
                outFile.write((char*)&subChunk2Size,sizeof(subChunk2Size));

                //Write data with saturation
                for (unsigned int i= 0; i < vect.size(); i++)
                {
                    //Saturation
                    //short data = (short) max((float)SHRT_MIN,min((float)SHRT_MAX,vect[i] * (float)(SHRT_MAX)));
                    short data = (short) (vect[i] * 32768.0);

		    //ROS_INFO("Writing : %i",data);
                    outFile.write((char*) &data,sizeof(short));
                }

                //Close file
                outFile.close();

                //Remove source
                streamData_.erase(id);
            }
        }

        void audio_source_cb(const manyears_msgs::ManyEarsTrackedAudioSourceConstPtr &sources)
        {
            std::list<int> removed_sources;

            //By default, every source will be saved and deleted
            for ( std::map<int,std::vector<float> >::iterator iter = streamData_.begin();
                 iter != streamData_.end(); iter++)
            {
                removed_sources.push_back(iter->first);
            }

            //ROS_INFO("**** SEQUENCE = %i",sources->sequence);

            //Iterate through sources
            for (unsigned int i = 0; i < sources->tracked_sources.size(); i++)
            {
                //unique ID of the source
                int id = sources->tracked_sources[i].source_id;

                //Prevent source from being removed
                for (std::list<int>::iterator iter = removed_sources.begin();
                     iter != removed_sources.end(); iter++)
                {
                    if (*iter == id)
                    {
                        removed_sources.erase(iter);
                        break;
                    }
                }

                //Append data at the end of the stream
                for (int j = 0; j < sources->tracked_sources[i].separation_data.size(); j++)
                {
                    streamData_[id].push_back(sources->tracked_sources[i].separation_data[j]);
                }

                //ROS_INFO("Stream id : %i size : %i",id,streamData_[id].size());
            }


            //Remove terminated sources
            for (std::list<int>::iterator iter = removed_sources.begin();
                 iter != removed_sources.end(); iter++)
            {
                save_wave(*iter);
            }

        }

    protected:

        std::map<int,std::vector<float> > streamData_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        std::string topic_;
    };
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manyears_savestream");

    manyears_savestream_node::manyears_savestream my_savestream;

    ros::spin();

}
