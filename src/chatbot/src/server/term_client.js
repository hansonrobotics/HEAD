const Client = require('./public/client.js').Client;
const client = new Client();
client.exit = process.exit;
client.print = function(msg){
    process.stdout.write(msg);
}
let stdin;
client.do_conn(false, function(){
    process.stdout.write("[me]: ");
    stdin = process.openStdin();
    stdin.addListener("data", function(msg) {
        client.execute(String(msg));
        process.stdout.write("[me]: ");
        // note:  d is an object, and when converted to a string it will
        //     // end with a linefeed.  so we (rather crudely) account for that  
        //         // with toString() and then trim() 
    });
});
