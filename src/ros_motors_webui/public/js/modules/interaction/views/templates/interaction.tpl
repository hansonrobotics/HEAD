<div class="container content">
    <div class="row">
        <div class="col-md-12">
            <div class="app-language-select btn-group pull-right" role="group" aria-label="...">
                <button type="button" data-lang="en" class="btn btn-default btn-sm active">EN</button>
                <button type="button" data-lang="cn" class="btn btn-default btn-sm">CN</button>
            </div>
        </div>
    </div>
    <div class="row">
        <div class="col-md-12">
            <ul class="app-messages"></ul>
        </div>
    </div>
</div>

<footer class="navbar-default navbar-fixed-bottom">
    <div class="container">
        <div class="row">
            <div class="col-sm-12">
                <div class="record-container col-sm-12">
                    <div class="app-select-person-container">
                        <button class="expand-face-select-button btn btn-primary btn-sm" type="button"
                                data-toggle="collapse"
                                data-target=".app-face-container" aria-expanded="false" aria-controls="collapse">
                            Select a person
                        </button>

                        <div class="app-face-container collapse">
                            <div class="face-select-instruction">touch the image of your face, to talk to me</div>
                            <div class="app-face-thumbnails clearfix"></div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        <div class="row">
            <div class="col-sm-12">
                <div class="message-input-container input-group">
                    <input type="text" class="app-message-input form-control"
                           placeholder="Type your message here..."/>

                    <span class="input-group-btn">
                        <button class="app-send-button btn btn-primary">Send</button>
                        <button class="app-record-button btn btn-info"><i class="fa fa-microphone"></i></button>
                    </span>
                </div>
            </div>
        </div>
    </div>
</footer>
