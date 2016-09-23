<span class="chat-img <%= author == 'Robot' ? 'pull-right' : 'pull-left' %>">
    <% if (typeof type != 'undefined' && type == 'suggestion') { %>
    <span class="app-suggestion-actions">
        <i class="app-discard fa fa-times-circle"></i>
        <i class="app-accept fa fa-check-circle"></i>
    </span>
    <% } else if (author == 'Robot') { %>
    <img src="/u.gif" alt="User Avatar" class="img-circle"/>
    <% } else { %>
    <img src="/me.gif" alt="User Avatar" class="img-circle"/>
    <% } %>
</span>

<div class="chat-body clearfix">
    <% if (typeof type == 'undefined' || type != 'suggestion') { %>
        <div class="header">
            <strong class="primary-font name"></strong>
            <small class="pull-right text-muted">
                <span class="glyphicon glyphicon-time white"></span>
                <span class="time"><%= time %></span>
            </small>
        </div>
    <% } %>
    <p>
        <span class="app-shortcut-label span label label-success"></span>
        <span class="msg">
            <%= message %>
        </span>
    </p>
</div>
