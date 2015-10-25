<div class="container content">
    <ul class="app-messages"></ul>
</div>

<footer class="navbar-default navbar-fixed-bottom">
    <div class="container">
        <div class="row">
            <div class="col-sm-12">
                <div class="message-input-container input-group">
                    <input type="text" class="app-message-input form-control input-sm"
                           placeholder="Type your message here..."/>

                    <span class="input-group-btn">
                        <button class="app-send-button btn btn-primary btn-sm">Send</button>
                    </span>
                </div>

                <div class="record-container col-sm-12">
                    <button class="app-record-button btn btn-info btn-sm">
                        Say "Hi" to start!
                    </button>

                    <div class="app-select-person-container">
                        <button class="expand-face-select-button btn btn-primary btn-sm" type="button"
                                data-toggle="collapse"
                                data-target="#app-face-container" aria-expanded="false" aria-controls="collapse">
                            Select a person
                        </button>

                        <div id="app-face-container" class="collapse">
                            <div class="app-face-thumbnails clearfix"></div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>
</footer>

<div class="select-person-container container">
    <div class="row">
        <div class="col-md-12">

        </div>
    </div>
</div>
