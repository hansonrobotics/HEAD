<nav class="navbar navbar-default navbar-fixed-top" role="navigation">
    <div class="container">
        <div class="navbar-header">
            <button type="button" class="navbar-toggle collapsed" data-toggle="collapse"
                    data-target="#bs-example-navbar-collapse-1">
                <span class="sr-only">Toggle navigation</span>
                <span class="icon-bar"></span>
                <span class="icon-bar"></span>
                <span class="icon-bar"></span>
            </button>

            <a class="navbar-brand" href="#">
                <img id="logo" class="pull-left" src="assets/img/H.png" alt=""/>
                <span id="app-title"></span>
            </a>

            <span id="notifications" class="pull-left">
                <span id="app-connecting" class="label label-warning">Connecting... Please wait</span>
                <span id="app-connection-error" class="label label-danger" style="display: none">Connection error. Click to refresh</span>
            </span>
        </div>

        <div class="collapse navbar-collapse" id="bs-example-navbar-collapse-1">
            <ul class="nav navbar-nav navbar-right">
                <li><a id="app-status-link" href="#/status" class="app-change-page active"
                       data-page="#app-page-status">Status</a></li>
                <li><a id="app-expressions-link" href="#/expressions" class="app-change-page"
                       data-page="#app-page-expressions">Expressions</a></li>
                <li><a id="app-motors-link" href="#/motors" class="app-change-page"
                       data-page="#app-page-motors">Motors</a></li>
                <li><a id="app-animations-link" href="#/animations" class="app-change-page"
                       data-page="#app-page-animations">Animations</a></li>
                <li><a id="app-gestures-link" href="#/gestures" class="app-change-page" data-page="#app-page-gestures">Gestures</a>
                </li>
                <li><a id="app-interactions-link" href="#/interactions" class="app-change-page"
                       data-page="#app-page-interaction">Interaction</a>
                </li>
            </ul>
        </div>
    </div>
</nav>

<div id="app-content"></div>
