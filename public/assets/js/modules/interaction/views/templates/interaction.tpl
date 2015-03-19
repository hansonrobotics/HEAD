<div id="app-page-interaction" class="container content">
    <ul id="app-chat">
        <li id="leftMsg" class="left clearfix">
                <span class="chat-img pull-left">
                    <img src="assets/img/u.gif" alt="User Avatar" class="img-circle"/>
                </span>

            <div class="chat-body clearfix">
                <div class="header">
                    <strong class="primary-font name">Jack Sparrow</strong>
                    <small class="pull-right text-muted">
                        <span class="glyphicon glyphicon-time white"></span><span class="time"></span></small>
                </div>
                <p class="msg"></p>
            </div>
        </li>
        <li id="rightMsg" class="right clearfix">
                <span class="chat-img pull-right">
                    <img src="assets/img/me.gif" alt="User Avatar" class="img-circle"/>
                </span>

            <div class="chat-body clearfix">
                <div class="header">
                    <small class=" text-muted"><span class="glyphicon glyphicon-time white"></span><span
                            class="time">15 mins ago</span>
                    </small>
                    <strong class="pull-right primary-font name"></strong>
                </div>
                <p class='msg'></p>
            </div>
        </li>
    </ul>
</div>

<footer class="navbar-default navbar-fixed-bottom">
    <div class="container">
        <div class="row">
            <div class="col-sm-12">
                <div id="message-input-container" class="input-group">
                    <input id="app-message-input" type="text" class="form-control input-sm"
                           placeholder="Type your message here..."/>

                        <span class="input-group-btn">
                            <button class="btn btn-primary btn-sm" id="app-send-button">Send</button>
                        </span>
                </div>

                <div id="record-container" class="col-sm-12">
                    <button class="btn btn-info btn-sm" id="app-record-button">
                        Start Recording
                    </button>
                </div>
            </div>
        </div>
    </div>
</footer>
