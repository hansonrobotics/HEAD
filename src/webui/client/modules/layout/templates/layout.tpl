<nav id="navbar" class="navbar navbar-default navbar-fixed-top" role="navigation">
    <div id="navbar-container" class="container">
        <div class="navbar-header">
            <a class="navbar-brand" href="#">
                <img id="logo" class="pull-left" src="/H.png" alt=""/>
                <span id="app-title"></span>
            </a>
            <span id="notifications" class="pull-left">
                <span id="app-connecting" class="label label-warning">Connecting... Please wait</span>
                <span id="app-connection-error" class="label label-danger" style="display: none">Connection error. Click to refresh</span>
            </span>
            <button type="button" class="navbar-toggle collapsed" data-toggle="collapse"
                    data-target="#app-navigation">
                <span class="sr-only">Toggle navigation</span>
                <span class="icon-bar"></span>
                <span class="icon-bar"></span>
                <span class="icon-bar"></span>
            </button>
        </div>
        <div id="app-nav" class="collapse navbar-collapse">
            <ul id="app-user-nav" class="nav navbar-nav navbar-right">
                <li><a href="#/puppeteering">Puppeteering</a></li>
                <li><a href="#/performances">Performances</a></li>
                <!-- <li><a href="#/attention">Attention Regions</a></li> -->
                <li><a href="#/gestures">Gestures</a></li>
                <li><a href="#/expressions">Expressions</a></li>
                <li><a href="#/motors">Motors</a></li>
                <li><a href="#/interactions">Chat</a></li>
                <li>
                    <a href="#" title="Report a bug" class="app-report-button"><i class="fa fa-exclamation-circle"></i></a>
                </li>
            </ul>
            <ul id="app-admin-nav" class="nav navbar-nav navbar-right">
                <li><a href="#/admin/monitor">Monitoring</a></li>
                <li><a href="#/admin/animations">Animations</a></li>
                <li><a href="#/admin/expressions">Expressions</a></li>
                <li><a href="#/admin/motors">Motors</a></li>
                <li><a href="#/admin/settings">Settings</a></li>
                <li>
                    <a href="#" title="Report a bug" class="app-report-button"><i class="fa fa-exclamation-circle"></i></a>
                </li>
            </ul>
        </div>
    </div>
</nav>

<div id="app-content"></div>
