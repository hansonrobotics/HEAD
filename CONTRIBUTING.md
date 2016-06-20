Welcome to contribute code to HEAD and make it even better and greater. Here are some things you need to know. 
 
 - [Found an Issue](#issue)
 - [Get Your Code Ready](#code)
 - [New Dependency](#dep)
 - [Logging](#log)
 - [Repository](#rep)

## <a name="issue"></a>Found an Issue
Please use the [GitHub issue tracker](https://github.com/hansonrobotics/HEAD/issues) to report an issue. If it's really critical issue, please lable it as "critical bug".

## <a name="code"></a>Get Your Code Ready
* Your code should conform the code style commonly adopted in the industry. 
* Your code should be tested. Preferably with unit tests. 

## <a name="dep"></a>New Dependency

The preferred way to install the dependencies is [apt] > [pip] > [source code].

If we build from source code, I suggest we always set the prefix to something like /opt/HEAD. There are two reasons.

* It won’t mess up user’s development environment. If you can’t find the package from the distribution, the compatibility of this library is probably not tested. Install it to “/usr” or “/usr/local” will have the risk to break user’s development environment.
* Make future release easier. If we want to release the software, it’ll be easier to package if the dependencies are all located in the same path. It’ll also be easier to uninstall everything.

## <a name="log"></a>Logging
There is a common place for logging. That is `~/.hr/log`.

The format of logging is `[%(name)s][%(levelname)s] %(asctime)s: %(message)s`

For example
`[rospy.impl.masterslave][INFO] 2016-04-19 14:04:11,842: atexit`

## <a name="rep"></a>Repository
It’s generally not recommended to clone the repository to workspace just because it’s a dependency. But if we need to work on it to customize it, it’s an exception. 
