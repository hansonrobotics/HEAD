require('./public/client.js');
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
    });
});
