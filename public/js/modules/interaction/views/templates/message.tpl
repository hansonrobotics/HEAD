<span class="chat-img <%= author == 'Robot' ? 'pull-right' : 'pull-left' %>">
    <% if (author == 'Robot') { %>
    <img src="/public/img/u.gif" alt="User Avatar" class="img-circle"/>
    <% } else { %>
    <img src="/public/img/me.gif" alt="User Avatar" class="img-circle"/>
    <% } %>
</span>

<div class="chat-body clearfix">
    <div class="header">
        <strong class="primary-font name"></strong>
        <small class="pull-right text-muted">
            <span class="glyphicon glyphicon-time white"></span>
            <span class="time"><%= time %></span>
        </small>
    </div>
    <p class="msg"><%= message %></p>
</div>
