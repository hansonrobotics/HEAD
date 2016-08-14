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

    const line = document.createElement("span");
    line.classList = [tag];
    line.innerHTML = message;

    messageHistory.appendChild(line);
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

function download(text, fname, file){
    const elem = document.createElement("a");
    const file = new Blob([txt], {type: type});
    elem.href = URL.createObjectURL(file);
    elem.download = name;
    elem.innerText = "Click here to download";
    messageHistory.appendChild(elem);
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
client.download = download;

messageBox.addEventListener("keydown", keypress);
