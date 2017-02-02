# Performances Engine
ROS Control for performances created in tiumeline editor.
Performances are saved in robots_config package {robot_name}/performances folder.

## Nodes
 * Performances runner: [runner.py](/scripts/runner.py)
 * Wholeshow: [wholeshow.py](/scripts/runner.py)

## Performance engine
### Services
All services can by run with /performances prefix.
  * /run ([Trigger](http://docs.ros.org/indigo/api/std_srvs/html/srv/Trigger.html))

    Runs current loaded performance

  * /run_by_name ([RunByName](/srv/RunByName.srv))

    Runs single performance by its path in performances folder

    `id:` Path of the single performance. ".yaml" part to be ommited.

    Returns:

    `success`: True if performance started.


  * /run_full_performance ([RunByName](/srv/RunByName.srv))

    Runs all performances by their name in the specified performance
    folder. If Folder contains one or more folders it randomly picks one
    of those folders to run.

    `id:` Path of the performance.

    Returns:

    `success`: True if performance started.

  * /resume ([Resume](/srv/Resume.srv))

   Resumes currently loaded performance

   Returns:

   `success`: True if performance started.

   `time`: Time at which performance resumed.

  * /stop ([Stop](/srv/Stop.srv))

   Stops current performance

   Returns:

   `success`: True if performance stopped.

   `time`: Time at which performance was stopped.

  * /pause ([Pause](/srv/Pause.srv))

   Pauses current performance

   Returns:

   `success`: True if performance paused.

   `time`: Time at which performance was paused.

  * /load ([Load](/srv/Load.srv))

   Returns performance performance as JSON string from its id.

   `id:` Path of the performance.

   Returns:

   `success`: True if performance found and it was legal syntax.

   `time`: Time at which performance was paused.
   `performance`: JSON string of the performance file

  * /load ([Resume](/srv/Load.srv))

   Returns performance performance as JSON string from its id.

   `id:` Path of the performance.

   Returns:

   `success`: True if performance found and it was legal syntax.

   `performance`: JSON string of the performance file

  * /load_performance ([Resume](/srv/LoadPerformance.srv))

   Loads performance from string as the current performance.

   `performance:` JSON formatted performance

   Returns:

   `success`: True if performance parsed and loaded.

  * /load_sequence ([Resume](/srv/LoadSequence.srv))

   Combines and loads multiple performances  as current and returns
   as single performance.

   `ids:`: Array of ids of performances to be loaded

   Returns:

   `performances`: JSON string of combined performances

   `success`: True if performance parsed and loaded.
