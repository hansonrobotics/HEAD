/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <ros_open_ear/smileCommon.hpp>

#include <ros_open_ear/configManager.hpp>
#include <ros_open_ear/commandlineParser.hpp>
#include <ros_open_ear/componentManager.hpp>

#define MODULE "SMILExtract"


/************** Ctrl+C signal handler **/
#include  <signal.h>

cComponentManager *cmanGlob = NULL;

void INThandler(int);
int ctrlc = 0;

void INThandler(int sig)
{
  signal(sig, SIG_IGN);
  if (cmanGlob != NULL) cmanGlob->requestAbort();
  signal(SIGINT, INThandler);
  ctrlc = 1;
}

#include <sstream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "emo_talker");


  try {

    // set up the smile logger
    LOGGER.setLogLevel(1);
    LOGGER.enableConsoleOutput();


    // commandline parser:
    cCommandlineParser cmdline(argc,argv);
    cmdline.addStr( "configfile", 'C', "Path to openSMILE config file", "smile.conf" );
    cmdline.addInt( "loglevel", 'l', "Verbosity level (0-9)", 2 );
#ifdef DEBUG
    cmdline.addBoolean( "debug", 'd', "Show debug messages (on/off)", 0 );
#endif
    cmdline.addInt( "nticks", 't', "Number of ticks to process (-1 = infinite) (only works for single thread processing, i.e. nThreads=1)", -1 );
    //cmdline.addBoolean( "configHelp", 'H', "Show documentation of registered config types (on/off)", 0 );
    cmdline.addBoolean( "components", 'L', "Show component list", 0 );
    cmdline.addStr( "configHelp", 'H', "Show documentation of registered config types (on/off/argument) (if an argument is given, show only documentation for config types beginning with the name given in the argument)", NULL, 0 );
    cmdline.addBoolean( "ccmdHelp", 'c', "Show custom commandline option help (those specified in config file)", 0 );
    cmdline.addStr( "logfile", 0, "set log file", "smile.log" );
    cmdline.addBoolean( "nologfile", 0, "don't write to a log file (e.g. on a read-only filesystem)", 0 );
    cmdline.addBoolean( "noconsoleoutput", 0, "don't output any messages to the console (log file is not affected by this option)", 0 );
    cmdline.addBoolean( "appendLogfile", 0, "append log messages to an existing logfile instead of overwriting the logfile at every start", 0 );

    int help = 0;
    if (cmdline.doParse() == -1) {
      LOGGER.setLogLevel(0);
      help = 1;
    }
    if (argc <= 1) {
      printf("\nNo commandline options were given.\n Please run ' SMILExtract -h ' to see some usage information!\n\n");
      return 10;
    }

    if (cmdline.getBoolean("nologfile")) {
      LOGGER.setLogFile((const char *)NULL,0,!(cmdline.getBoolean("noconsoleoutput")));
    } else {
      LOGGER.setLogFile(cmdline.getStr("logfile"),cmdline.getBoolean("appendLogfile"),!(cmdline.getBoolean("noconsoleoutput")));
    }
    LOGGER.setLogLevel(cmdline.getInt("loglevel"));
    SMILE_MSG(2,"openSMILE starting!");

#ifdef DEBUG  // ??
    if (!cmdline.getBoolean("debug"))
      LOGGER.setLogLevel(LOG_DEBUG, 0);
#endif

    SMILE_MSG(2,"config file is: %s",cmdline.getStr("configfile"));



    // create configManager:
    cConfigManager *configManager = new cConfigManager(&cmdline);

    cComponentManager *cMan = new cComponentManager(configManager,componentlist);
    const char *selStr=NULL;
    if (cmdline.isSet("configHelp")) {
      selStr = cmdline.getStr("configHelp");
      configManager->printTypeHelp(0,selStr);
      help = 1;
    }
    if (cmdline.getBoolean("components")) {
      cMan->printComponentList();
      help = 1;
    }

    if (help==1) {
      delete configManager;
      delete cMan;
      return -1;
    }

    // TODO: read config here and print ccmdHelp...
    // add the file config reader:
    configManager->addReader( new cFileConfigReader( cmdline.getStr("configfile") ) );
    configManager->readConfig();

    /* re-parse the command-line to include options created in the config file */
    cmdline.doParse(1,0); // warn if unknown options are detected on the commandline
    if (cmdline.getBoolean("ccmdHelp")) {
      cmdline.showUsage();
      delete configManager;
      delete cMan;
      return -1;
    }

    /* create all instances specified in the config file */
    cMan->createInstances(0); // 0 = do not read config (we already did that above..)

    /*
    MAIN TICK LOOP :
    */
    cmanGlob = cMan;
    signal(SIGINT, INThandler); // install Ctrl+C signal handler

    /* run single or mutli-threaded, depending on componentManager config in config file */
    long long nTicks = cMan->runMultiThreaded(cmdline.getInt("nticks"));

    /* it is important that configManager is deleted BEFORE componentManger!
      (since component Manger unregisters plugin Dlls, which might have allocated configTypes, etc.) */
    delete configManager;
    delete cMan;

  } catch(cSMILException *c) {
    // free exception ??
    return EXIT_ERROR;
  }

  if (ctrlc) return EXIT_CTRLC;
  return EXIT_SUCCESS;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int oldMain(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  // %Tag(LOOP_RATE)%
    ros::Rate loop_rate(10);
  // %EndTag(LOOP_RATE)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// %EndTag(PUBLISHER)%



  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
