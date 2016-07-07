#include "manyears_config.hpp"
#include <ros/ros.h>

using namespace manyears_ros;

bool manyears_ros::parseConfigFile(objOverall& context, const std::string& fn)
{
    // NOTE: Taken directly from manyears_ros with minor fixes.
    
    const int BUFFER_SIZE = 1024;
    char current_line[BUFFER_SIZE];
    std::string current_str;
    FILE* file_param_ptr;
    file_param_ptr = fopen(fn.c_str(), "rb");
    if(file_param_ptr == NULL) {
        ROS_ERROR("Cannot open file '%s'", fn.c_str());
        return false;
    } else {
        while(fgets(current_line, BUFFER_SIZE, file_param_ptr) != NULL) {
            current_str.assign(current_line);
            int equal_pos = current_str.find("=");
            std::string key_str = current_str.substr(0, equal_pos);
            std::string value_str = 
                current_str.substr(equal_pos + 2,
                                   current_str.size() - equal_pos - 4);
            std::istringstream stream_val (value_str);
            float val;
            stream_val >> val;
            ROS_DEBUG("key %s, value %s, val %f",
                      key_str.c_str(), 
                      value_str.c_str(),
                      val);
            loadParameter(context, key_str, val);
        }
        fclose(file_param_ptr);
    }

    return true;
}

void manyears_ros::loadParameter(      objOverall&  context, 
                                 const std::string& key,
                                       float        val)
{
    if(key.compare("BEAMFORMER_MAXSOURCES") == 0){
        context.myParameters->P_BF_MAXSOURCES = (int)val;
        //ROS_INFO("final : %d", context.myParameters->P_BF_MAXSOURCES );
    }
    if(key.compare("BEAMFORMER_ET") == 0)
        context.myParameters->P_BF_ET = (float)val;
    if(key.compare("BEAMFORMER_FILTERRANGE") == 0)
        context.myParameters->P_BF_FILTERRANGE = (float)val;
    if(key.compare("BEAMFORMER_RESETRANGE") == 0)
        context.myParameters->P_BF_RESETRANGE = (float)val;
    if(key.compare("GEO_MICS_MIC1_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC1_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC1_X") == 0)
        context.myParameters->P_GEO_MICS_MIC1_X = (float)val;
    if(key.compare("GEO_MICS_MIC1_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC1_Y = (float)val;
    if(key.compare("GEO_MICS_MIC1_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC1_Z = (float)val;
    if(key.compare("GEO_MICS_MIC2_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC2_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC2_X") == 0)
        context.myParameters->P_GEO_MICS_MIC2_X = (float)val;
    if(key.compare("GEO_MICS_MIC2_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC2_Y = (float)val;
    if(key.compare("GEO_MICS_MIC2_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC2_Z = (float)val;
    if(key.compare("GEO_MICS_MIC3_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC3_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC3_X") == 0)
        context.myParameters->P_GEO_MICS_MIC3_X = (float)val;
    if(key.compare("GEO_MICS_MIC3_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC3_Y = (float)val;
    if(key.compare("GEO_MICS_MIC3_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC3_Z = (float)val;
    if(key.compare("GEO_MICS_MIC4_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC4_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC4_X") == 0)
        context.myParameters->P_GEO_MICS_MIC4_X = (float)val;
    if(key.compare("GEO_MICS_MIC4_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC4_Y = (float)val;
    if(key.compare("GEO_MICS_MIC4_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC4_Z = (float)val;
    if(key.compare("GEO_MICS_MIC5_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC5_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC5_X") == 0)
        context.myParameters->P_GEO_MICS_MIC5_X = (float)val;
    if(key.compare("GEO_MICS_MIC5_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC5_Y = (float)val;
    if(key.compare("GEO_MICS_MIC5_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC5_Z = (float)val;
    if(key.compare("GEO_MICS_MIC6_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC6_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC6_X") == 0)
        context.myParameters->P_GEO_MICS_MIC6_X = (float)val;
    if(key.compare("GEO_MICS_MIC6_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC6_Y = (float)val;
    if(key.compare("GEO_MICS_MIC6_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC6_Z = (float)val;
    if(key.compare("GEO_MICS_MIC7_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC7_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC7_X") == 0)
        context.myParameters->P_GEO_MICS_MIC7_X = (float)val;
    if(key.compare("GEO_MICS_MIC7_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC7_Y = (float)val;
    if(key.compare("GEO_MICS_MIC7_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC7_Z = (float)val;
    if(key.compare("GEO_MICS_MIC8_GAIN") == 0)
        context.myParameters->P_GEO_MICS_MIC8_GAIN = (float)val;
    if(key.compare("GEO_MICS_MIC8_X") == 0)
        context.myParameters->P_GEO_MICS_MIC8_X = (float)val;
    if(key.compare("GEO_MICS_MIC8_Y") == 0)
        context.myParameters->P_GEO_MICS_MIC8_Y = (float)val;
    if(key.compare("GEO_MICS_MIC8_Z") == 0)
        context.myParameters->P_GEO_MICS_MIC8_Z = (float)val;

    // Filter
    if(key.compare("FILTER_STANDARDDEVIATION") == 0)
        context.myParameters->P_FILTER_STDDEVIATION = (float)val;
    if(key.compare("FILTER_PREDICTION_STOP_ALPHA") == 0)
        context.myParameters->P_FILTER_ALPHASTOP = (float)val;
    if(key.compare("FILTER_PREDICTION_STOP_BETA") == 0)
        context.myParameters->P_FILTER_BETASTOP = (float)val;
    if(key.compare("FILTER_PREDICTION_CONSTANT_ALPHA") == 0)
        context.myParameters->P_FILTER_ALPHACONST = (float)val;
    if(key.compare("FILTER_PREDICTION_CONSTANT_BETA") == 0)
        context.myParameters->P_FILTER_BETACONST = (float)val;
    if(key.compare("FILTER_PREDICTION_ACCELERATED_ALPHA") == 0)
        context.myParameters->P_FILTER_ALPHAEXC = (float)val;
    if(key.compare("FILTER_PREDICTION_ACCELERATED_BETA") == 0)
        context.myParameters->P_FILTER_BETAEXC = (float)val;
    if(key.compare("FILTER_INERTIA_X") == 0)
        context.myParameters->P_FILTER_INERTIAX = (float)val;
    if(key.compare("FILTER_INERTIA_Y") == 0)
        context.myParameters->P_FILTER_INERTIAY = (float)val;
    if(key.compare("FILTER_INERTIA_Z") == 0)
        context.myParameters->P_FILTER_INERTIAZ = (float)val;
    if(key.compare("FILTER_DELTAT") == 0)
        context.myParameters->P_FILTER_DELTAT = (float)val;
    if(key.compare("FILTER_STATEUPDATE") == 0)
        context.myParameters->P_FILTER_STATEUPDT = (float)val;
    if(key.compare("FILTER_STOP_PERCENTAGE") == 0)
        context.myParameters->P_FILTER_NEWSTOP = (float)val;
    if(key.compare("FILTER_CONSTANT_PERCENTAGE") == 0)
        context.myParameters->P_FILTER_NEWCONST = (float)val;
    if(key.compare("FILTER_ACCELERATED_PERCENTAGE") == 0)
        context.myParameters->P_FILTER_NEWEXC = (float)val;
    if(key.compare("FILTER_ACTIVE_ACTIVE") == 0)
        context.myParameters->P_FILTER_AJT_AJTM1 = (float)val;
    if(key.compare("FILTER_INACTIVE_ACTIVE") == 0)
        context.myParameters->P_FILTER_AJT_NOTAJTM1 = (float)val;
    if(key.compare("FILTER_P0") == 0)
        context.myParameters->P_FILTER_P0 = (float)val;
    if(key.compare("FILTER_RESAMPLING_THRESHOLD") == 0)
        context.myParameters->P_FILTER_RSTHRESHOLD = (float)val;
    if(key.compare("FILTER_BUFFERSIZE") == 0)
        context.myParameters->P_FILTER_BUFFERSIZE = (int)val;

    // Mixture
    if(key.compare("GEN_DYNSOURCES") == 0)
        context.myParameters->P_GEN_DYNSOURCES = (int)val;
    if(key.compare("MIXTURE_PNEW") == 0)
        context.myParameters->P_MIXTURE_PNEW = (float)val;
    if(key.compare("MIXTURE_PFALSE") == 0)
        context.myParameters->P_MIXTURE_PFALSE = (float)val;
    if(key.compare("MIXTURE_NEWSOURCE_THRESHOLD") == 0)
        context.myParameters->P_MIXTURE_NEWTHRESHOLD = (float)val;
    if(key.compare("MIXTURE_CONFIRM_SOURCE_EXISTS") == 0)
        context.myParameters->P_MIXTURE_CONFIRMEXISTS = (float)val;
    if(key.compare("MIXTURE_CONFIRM_COUNT_THRESHOLD") == 0)
        context.myParameters->P_MIXTURE_CONFIRMCOUNTTS = (float)val;
    if(key.compare("MIXTURE_CONFIRM_COUNT_COUNTER") == 0)
        context.myParameters->P_MIXTURE_CONFIRMCOUNT = (int)val;
    if(key.compare("MIXTURE_NEWSOURCE_HORIZONTALANGLE") == 0)
        context.myParameters->P_MIXTURE_NEWANGLE = (float)val;
    if(key.compare("MIXTURE_CUMULATIVE_TIME_PROBATION") == 0)
        context.myParameters->P_MIXTURE_CUMULATIVETIMEPROB = (int)val;
    if(key.compare("MIXTURE_NOTOBSERVED_PROBATION_THRESHOLD") == 0)
        context.myParameters->P_MIXTURE_TOBSPROB = (float)val;
    if(key.compare("MIXTURE_CUMULATIVE_TIME_LEVEL1") == 0)
        context.myParameters->P_MIXTURE_CUMULATIVETIME1 = (int)val;
    if(key.compare("MIXTURE_NOTOBSERVED_LEVEL1_THRESHOLD") == 0)
        context.myParameters->P_MIXTURE_TOBS1 = (float)val;
    if(key.compare("MIXTURE_CUMULATIVE_TIME_LEVEL2") == 0)
        context.myParameters->P_MIXTURE_CUMULATIVETIME2 = (int)val;
    if(key.compare("MIXTURE_NOTOBSERVED_LEVEL2_THRESHOLD") == 0)
        context.myParameters->P_MIXTURE_TOBS2 = (float)val;

    // Microphone Sound Track
    if(key.compare("MICST_ALPHAD") == 0)
        context.myParameters->P_MICST_ALPHAD = (float)val;
    if(key.compare("MICST_GAMMA") == 0)
        context.myParameters->P_MICST_GAMMA = (float)val;
    if(key.compare("MICST_DELTA") == 0)
        context.myParameters->P_MICST_DELTA = (float)val;
    if(key.compare("MICST_MCRA_ALPHAS") == 0)
        context.myParameters->P_MCRA_ALPHAS = (float)val;
    if(key.compare("MICST_MCRA_ALPHAP") == 0)
        context.myParameters->P_MCRA_ALPHAP = (float)val;
    if(key.compare("MICST_MCRA_ALPHAD") == 0)
        context.myParameters->P_MCRA_ALPHAD = (float)val;
    if(key.compare("MICST_MCRA_L") == 0)
        context.myParameters->P_MCRA_L = (int)val;
    if(key.compare("MICST_MCRA_DELTA") == 0)
        context.myParameters->P_MCRA_DELTA = (float)val;

    // Output
    if (key.compare("P_OUT_GAIN") == 0)
        context.myParameters->P_OUT_GAIN = (float)val;
}
