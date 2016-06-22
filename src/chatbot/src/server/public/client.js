'use strict'
var node = false;
/* For pre-IE9 compatibility*/
if(typeof(String.prototype.trim) === "undefined"){
    String.prototype.trim = function() {
        return String(this).replace(/^\s+|\s+$/g, '');
    };
}

Client.VERSION = 'v1.1';
Client.KEY = 'AAAAB3NzaC';
Client.URLEncodeJSON = function(params){
    let result = '';
    for(const strkey in params){
        const key = encodeURI(strkey);
        const val = encodeURI(params[strkey]);
        if(val === undefined || val === 'undefined')
            continue;
        if(result.indexOf('?') !== -1){
            result+='&' + key + '=' + val;
        }else{
            result+='?' + key + '=' + val;
        }
    }
    return result;
}

/* Gross get requests for JSON retrieval. In the future might want to use
   JQuery.
   */
Client.get = function(url, params){
    if(node)XMLHttpRequest = global.XMLHttpRequest;//For testing

    url += Client.URLEncodeJSON(params);

    const HTTP_REQ = new XMLHttpRequest();
    HTTP_REQ.open('GET',url,false);//TODO make this asynchronous
    HTTP_REQ.send(null);
    return HTTP_REQ;
}

Client.post = function(url, params, file){
    if(node)XMLHttpRequest = global.XMLHttpRequest;//For testing

    const formData = new FormData();

    formData.append('zipfile',file);
    for(const key in params){
        formData.append(key,params[key]);
    }

    const HTTP_REQ = new XMLHttpRequest();
    HTTP_REQ.open('POST', url, false);
    HTTP_REQ.send(formData);
    return HTTP_REQ;
}

function Client(auto_connect){
    const PROMPT = '[me]: ';

    let session = undefined;
    let lang = 'en';
    let chatbot_port = '8001';
    let chatbot_ip = 'localhost';
    let chatbot_url = 'http://' + chatbot_ip + ':' + chatbot_port + '/' + Client.VERSION;
    let bot_name = 'sophia';
    let user = 'client';
    const self = this;

    this.print = function(msg){
        console.log(msg);
    };
    this.println = function(msg){
        self.print(msg);
        self.print('\n');
    }
    this.error = function(msg){
        console.error(msg);
    };//If we want to change the write function

    this.exit = function(){
        self.println("Bye");
    };
    //Callback to be set outside of the function.

    let exec = function(cmd,param){
        const f_name= 'do_' + cmd;
        let fn = self[f_name];
        fn(param);
    }

    this.execute = function(message){
        message = message.trim();

        let cmd, param;

        if(message.indexOf(' ') === -1){
            cmd = message;
            param = '';
        }else{
            cmd = message.substr(0,message.indexOf(' '));
            param = message.substr(message.indexOf(' ')+1);
        }

        if(COMMANDS.indexOf(cmd) !== -1){
            exec(cmd, param);
        }else{
            self.default(message + '\n');
        }
    }

    let finish_set_sid = function(response, callback, error_callback){
        const text = response.responseText;
        const status_code = response.status;

        if(status_code != 200){
            self.error("Request error: " + status_code);

            if(error_callback){
                //If there is a parameter to error_callback, it was a request
                //error
                error_callback(status_code);
            }

            return;
        }
        
        const json = JSON.parse(text);
        const ret = json['ret'];

        session = json['sid'];

        self.println("New session " + session);

        if(callback)
            callback(session);
    }

    this.set_sid = function(callback, error_callback){
        const params = {
            'Auth': Client.KEY,
            'botname': '' + bot_name,
            'user': user
        };

        const url = chatbot_url + '/start_session';

        self.println("Attempting to start new session.");
        
        let tries = 3;
        let loop = setInterval(function(){
            if(tries < 1){
                clearInterval(loop);

                self.error("Failed to start new session.");

                if(error_callback){
                    //If there is not a parameter to error_callback, it was
                    //a connection error.
                    error_callback();
                }

                return;
            }

            let attempt = {};
            try{
                attempt = Client.get(url, params);
            }catch(e){
                tries--;
                self.error(e);
                self.error('Connection error. ' + tries + (tries===1?' try':' tries') + ' left.');
                return;
            }
            clearInterval(loop);
            finish_set_sid(attempt, callback, error_callback);
        },1000);
    }

    this.ask = function(question){
        const params = {
            'question': '' + question,
            'session': session,
            'lang': lang,
            'Auth': Client.KEY
        };

        const url = chatbot_url + '/chat';
        const response = Client.get(url,params);
        const text = response.responseText;
        const status_code = response.status;

        if(status_code != 200){
            const err = "Request error: " + status_code + "\n";
            self.error(err);
            return;
        }

        const json = JSON.parse(text);
        const ret = json['ret'];

        if(ret != 0){
            const err = "QA error: error code " + ret + ", botname " + bot_name +
                ", question " + question + ", lang " + lang;
            self.error(err);
        }

        let result = {
            'text': '',
            'emotion': '',
            'botid': '',
            'botname': ''
        };

        for(const key in json['response']){
            result[key] = json['response'][key];
        }

        return [ret, result];
    }

    this.list_chatbot = function(){
        const params = {
            'Auth':Client.KEY,
            'lang':lang,
            'session': session
        };

        const response = Client.get(chatbot_url + '/chatbots',params).responseText;
        const chatbots = JSON.parse(response)['response'];

        return chatbots;
    }
    
    this.list_chatbot_names = function(){
        const params = {
            'Auth':Client.KEY,
            'lang':lang,
            'session':session
        };
        const response = Client.get(chatbot_url + '/bot_names', params).responseText;
        const names = JSON.parse(response)['response'];
        
        return names;
    }

    this.default = function(line){
        if(!line)
            return;

        try{
            let ask = self.ask(line);
            let ret = ask[0];
            let response = ask[1];
            if(ret != 0){
                self.set_sid(function(){
                    ask = self.ask(line);
                    ret = ask[0];
                    response = ask[1];
                    const message = bot_name + '[by ' + response['botname'] + ']: ' + response['text'];
                    self.println(message);
                });
                return;
            }
            const message = bot_name + '[by ' + response['botname'] + ']: ' + response['text'];
            self.println(message);
        }catch(e){
            self.error(e);
        }
    }

    this.do_list = function(line){
        let chatbots = [];
        try{
            chatbots = self.list_chatbot();
            let chatbots_str = "";

            for(let idx = 0; idx < chatbots.length; idx++){
                const bot = chatbots[idx];
                const this_bot_name = bot[0];//c
                const w = bot[1];//w
                const l = bot[2];//l

                const out = this_bot_name + 
                    ': weight: ' + w +
                    ' level: ' + l;

                chatbots_str += out;
                chatbots_str += '\n';
            }

            self.println(chatbots_str);
        }catch(e){
            self.error(e);
        }
    }

    this.help_list = function(){
        self.println("List chatbot names");
    }

    this.do_l = this.do_list;
    this.help_l = this.help_list;

    this.do_select = function(line, callback, error_callback){
        try{
            const names = self.list_chatbot_names();

            for(let idx = 0; idx < names.length; idx++){
                const name = names[idx];
                if(line === name){
                    bot_name = line;
                    self.set_sid(function(){
                        self.println("Select chatbot " + bot_name);

                        if(callback){
                            callback();
                        }
                    }, error_callback);
                    return;
                }
            }

            self.println("No such chatbot " + line);
            if(callback){
                callback();
            }
        }catch(e){
            self.error(e);
            return;
        }
    }

    this.help_select = function(){
        self.println("Select chatbot");
    }

    this.do_conn = function(line, callback, error_callback){
        if(line){
            try{
                const url = line.split(':');
                chatbot_ip = url[0];
                chatbot_port = url[1];
                chatbot_url = 'http://' +
                    chatbot_ip + ':' +
                    chatbot_port + '/' +
                    Client.VERSION;
            }catch(e){
                self.error("Wrong conn arguments\n");
                self.help_conn();
                return;
            }
        }
        self.println('Connecting.');
        self.set_sid(callback, error_callback);
    }

    this.help_conn = function(){
        const s = "\n"+
            "Connect to chatbot server\n"+
            "Syntax: conn [url:port]\n"+
            "For example, conn 127.0.0.1:8001\n";
        self.println(s);
    }

    this.do_ip = function(line){
        if(!line){
            self.println("ip is currently " + chatbot_ip);
            return;
        }

        chatbot_ip = line;
        chatbot_url = 'http://' + chatbot_ip + ':' + chatbot_port + '/' + Client.VERSION;
        self.println("ip is now " + chatbot_ip);
        self.println("url is now " + chatbot_url);
    }

    this.help_ip = function(){
        const s = "\n"+
            "Set the IP address of chatbot server\n"+
            "Syntax: ip xxx.xxx.xxx.xxx\n"+
            "For example, ip 127.0.0.1\n";
        self.println(s);
    }

    this.do_port = function(line){
        if(!line){
            self.println("port is currently " + chatbot_port);
            return;
        }

        chatbot_port = line;
        chatbot_url = 'http://' + chatbot_ip + ':' + chatbot_port + '/' + Client.VERSION;
        self.println("port is now " + chatbot_port);
        self.println("url is now " + chatbot_url);
    }

    this.help_port = function(){
        const s = "\n"+
            "Set the port of chatbot server\n"+
            "Syntax: port xxx\n"+
            "For example, port 8001\n";
        self.println(s);
    }

    this.do_q = function(){
        self.exit();
    }

    this.help_q = function(){
        self.println("Quits the chat\n");
    }

    let help = function(cmd){
        const f_name = 'help_' + cmd;
        let fn = self[f_name];

        if(fn === undefined){
            write_error('*** No help on ' + cmd);
            return;
        }

        fn();
    }

    let help_all = function(){
        let line_length = 45;
        let buffer = "Documented commands (type \"help [topic]\")\n"+
            "=============================================\n";
        let line = "";
        COMMANDS_WITH_HELP.forEach(function(val, idx){
            if(line.length + val.length > line_length){
                buffer += line + '\n';
                line = "";
            }
            line += val;
            line += " ";
        });
        buffer += line;
        self.println(buffer);
    }

    this.do_help = function(line){
        if(line.trim().length > 0){
            help(line);
        }else{
            help_all();
        }
    }

    this.do_lang = function(line){
        const new_lang = line.trim();
        if(new_lang === 'en' || new_lang === 'zh'){
            lang = new_lang;
            self.println("Set lang to " + lang);
        }else{
            self.println("Current lang " + lang + ". Set lang by \'lang [en|zh]\'");
        }
    }

    this.help_lang = function(){
        self.println("Set language. [en|zh]");
    }

    this.do_c = function(line){
        try{
            const params = {
                'session':session,
                'Auth':Client.KEY
            };
            const url = chatbot_url + '/reset_session';
            const response = Client.get(url,params);
            const text = response.responseText;
            const status_code = response.status;

            if(status_code != 200){
                const err = "Request error: " + status_code + "\n";
                self.error(err);
                return;
            }

            const json = JSON.parse(text);
            const ret = json['ret'];
            const res = json['response'];

            self.println(res);
        }catch(e){
            self.error(e);
        }
    }

    this.help_c = function(){
        self.println("Clean the memory of the dialog");
    }

    this.do_rw = function(line){
        try{
            const params = {
                'weights':line,
                'Auth':Client.KEY,
                'lang':lang,
                'session':session
            };
            const url = chatbot_url + '/set_weights';
            const response = Client.get(url,params);
            const text = response.responseText;
            const status_code = response.status;

            if(status_code != 200){
                const err = "Request error: " + status_code + "\n";
                self.error(err);
                return;
            }

            const json = JSON.parse(text);
            const ret = json['ret'];
            const res = json['response'];
            
            self.println(res);
        }catch(e){
            self.error(e);
        }
    }

    this.help_rw = function(){
        const s = "\n"+
            "Update the weights of the current chain.\n"+
            "Syntax: rw w1,w2,w3,...\n"+
            "For example, rw .2, .4, .5\n";
        self.println(s);
    }

    if(typeof document !== 'undefined'){
        const input = document.createElement("INPUT");
        input.setAttribute("type", "file");

        let prompt_for_upload = function(callback){
            input.onchange = function(e){
                callback(input.files[0]);
            };
            input.click();
        }

        this.do_upload = function(){
            prompt_for_upload(function(file){
                try{
                    const params = {
                        'user': user,
                        'Auth': Client.KEY,
                        'lang': 'en'
                    };

                    const url = chatbot_url + '/upload_character';
                    const response = Client.post(url, params, file);
                    const text = response.responseText;
                    const status_code = response.status;

                    if(status_code != 200){
                        const err = "Request error: " + status_code + "\n";
                        self.error(err);
                        return;
                    }

                    const json = JSON.parse(text);
                    const ret = json['ret'];
                    const res = json['response'];
                }catch(e){
                    self.error(e);
                }
            });
        }

        this.help_upload = function(){
            const msg = "Upload character package.\n" +
                "Syntax: upload package\n";
            self.println(msg);
        }
    }

    this.ping = function(){
        try{
            const url = chatbot_url + '/ping';
            const response = Client.get(url);
            const text = response.responseText;
            const status_code = response.status;

            if(status_code != 200){
                const err = "Request error: " + status_code + "\n";
                self.error(err);
                return;
            }

            const json = JSON.parse(text);
            const res = json['response'];
            if(res === 'pong'){
                return true;
            }
        }catch(e){
            return false;
        }
    }

    this.do_ping = function(line){
        if(self.ping){
            self.println('pong');
        }else{
            self.println('');
        }
    }

    this.get_PROMPT = function(){return PROMPT;}
    this.get_session = function(){return session;}
    this.get_lang = function(){return lang;}
    this.get_chatbot_port = function(){return chatbot_port;}
    this.get_chatbot_ip = function(){return chatbot_ip;}
    this.get_chatbot_url = function(){return chatbot_url;}
    this.get_bot_name = function(){return bot_name;}
    this.get_user = function(){return user;}

    const COMMANDS = [];
    for(const fn in self)
        if(fn.indexOf("do_") == 0)
            COMMANDS.push(fn.substr("do_".length));
    
    const COMMANDS_WITH_HELP = [];
    for(const fn in self)
        if(fn.indexOf("help_") == 0)
            COMMANDS_WITH_HELP.push(fn.substr("help_".length));
}

function useNodejs(){
    global.Client = Client;
    global.XMLHttpRequest = require('xmlhttprequest').XMLHttpRequest;
    XMLHttpRequest = global.XMLHttpRequest;
    node = true;
}

if(typeof exports !== 'undefined'){
    var XMLHttpRequest;//Scoping in case of Node.js
    useNodejs();
}
