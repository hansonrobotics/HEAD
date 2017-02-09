# Performance API

## Services:

#### `/performances/reload_properties`
`std_srvs.srv.Trigger`
Reads and loads performance configurations from .properties files.
##### Response
* `boolean success` 

#### `/performances/set_properties`
`performances.srv.SetProperties`
Sets variables for a specific performance.

##### Arguments
* `string id` - performance id
* `string properties` - a json string of an object containing key value pairs of variables to be set

##### Response
* `boolean success`

#### `/performances/load`
`performances.srv.Load`
Loads performance with the specified id into current session.

##### Arguments
* `id` - performance id

##### Response
* `boolean success`
* `string performance` - json data string of a loaded performance

#### `/performances/load_sequence`
`performances.srv.LoadSequence`
Loads a sequence of performances.

##### Arguments
* `string[] ids` - ids of performances to load

##### Response
* `boolean success`
* `string performances` - json data string of loaded performances

#### `/performances/load_performance`
`performances.srv.LoadPerformance`
Load performance data directly.

##### Arguments
* `string performance` - json data of a performance to load

##### Response
* `boolean success`

#### `/performances/run`
`performances.srv.Run`
Run currently loaded performance(s) at the given time.

##### Arguments
* `float64 startTime` - start time

##### Response
* `boolean success`

#### `/performances/run_by_name`
`performances.srv.RunByName`
Load performance by id and run immediately.

##### Arguments
* `string id` - performance id

##### Response
* `boolean success`

#### `/performances/run_full_performance`
`performances.srv.RunByName`
Load all performances in the specified directory and run immediately.

##### Arguments
* `string id` - path of the directory

##### Response
* `boolean success`

#### `/performances/resume`
`performances.srv.Resume`
Resume runner if paused.

##### Response
* `boolean success`
* `float64 time`
	
#### `/performances/pause`
`performances.srv.Pause`
Pause runner if currently running.

##### Response
* `boolean success`
* `float64 time`
	
#### `/performances/stop`
`performances.srv.Stop`
Stop runner if active.

##### Response
* `boolean success`
* `float64 time`
	
#### `/performances/current`
`performances.srv.Current`
Get information about the current state of runner.

##### Response
`string performances` - json array of currently loaded performances
`float32 current_time` - current run time
`bool running` - run status
`bool paused` - pause status
