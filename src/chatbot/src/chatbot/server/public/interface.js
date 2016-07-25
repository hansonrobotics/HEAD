const messageBox = document.getElementsByClassName("message")[0];
const messageHistory = document.getElementsByClassName("content")[0];

let client = new Client();
client.do_conn();

const MESSAGE_OUTPUT_TAG = 'message';
const ERROR_OUTPUT_TAG = 'error';
const USER_INPUT_TAG = 'user';

const COMMANDS = [];
for(const fn in client)
    if(fn.indexOf("do_") == 0)
        COMMANDS.push(fn.substr("do_".length));

function write(message, tag){
    if(!tag)
        tag='\'text\'';
    message = "" + message;
    message = message.split("\n").join("<br>");
    messageHistory.innerHTML += '<span class=' + tag + '>\n'+
                                message+'\n'+
                                '</span>'+'\n';
}

function write_error(message){
    write(message,ERROR_OUTPUT_TAG);
}

function write_user_input(message){
    write(message,USER_INPUT_TAG);
}

function exit(){
    client.println("Bye");
    client = null;
    write("Window is dead.", ERROR_OUTPUT_TAG);
}

function exec(cmd, param){
    const f_name = 'do_' + cmd;
    let fn = client[f_name];
    fn(param);
}

function execute(message){
    write_user_input(client.get_PROMPT() + message + '\n');
    client.execute(message);
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


client.print = write;
client.error = write_error;
client.exit = exit;

messageBox.addEventListener("keydown", keypress);
