const messageBox = document.getElementsByClassName("message")[0];
const messageHistory = document.getElementsByClassName("content")[0];
let client = new Client();

const MESSAGE_OUTPUT_TAG = 'message';
const ERROR_OUTPUT_TAG = 'error';
const USER_INPUT_TAG = 'user';

const COMMANDS = [];
for(fn in client)
    if(fn.indexOf("do_") == 0)
        COMMANDS.push(fn.substr("do_".length));

const COMMANDS_WITH_HELP = [];
for(fn in client)
    if(fn.indexOf("help_") == 0)
        COMMANDS_WITH_HELP.push(fn.substr("help_".length));

function write(message, tag){
    if(!tag)
        tag='\'text\'';
    message = "" + message;
    message = message.split("\n").join("<br>");
    messageHistory.innerHTML += '<div class=' + tag + '>\n'+
                                message+'\n'+
                                '</div>\n';
}

function write_error(message){
    write(message,ERROR_OUTPUT_TAG);
}

function write_user_input(message){
    write(message,USER_INPUT_TAG);
}

function doc_help(){
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
    write(buffer);
}

function help(cmd){
    const f_name = 'help_' + cmd;
    let fn = client[f_name];

    if(fn === undefined){
        write_error('*** No help on ' + cmd);
        return;
    }

    fn();
}

function exec(cmd, param){
    const f_name = 'do_' + cmd;
    let fn = client[f_name];
    fn(param);
}

function execute(message){
    write_user_input(client.get_PROMPT() + message);

    let cmd, param;

    if(message.indexOf(' ') === -1){
        cmd = message;
        param = '';
    }else{
        cmd = message.substr(0,message.indexOf(' '));
        param = message.substr(message.indexOf(' ')+1);
    }

    if(cmd === 'help'){
        if(param === '' || param === 'help'){
            doc_help();
        }else{
            help(param);
        }
    }else if(cmd === 'q'){
        write("Bye");
        client = null;//Free client for garbage collection
        write("Window is dead.", ERROR_OUTPUT_TAG);
    }else if(COMMANDS.indexOf(cmd) !== -1){
        exec(cmd, param);
    }else{
        client.default(message);
    }
}

function keypress(e){
    if(e.which != 13)
        return;

    const message = messageBox.value;
    if(message === '')
        return;

    execute(message);
    messageBox.value='';
}


client.write = write;
client.error = write_error;

messageBox.addEventListener("keydown", keypress);
