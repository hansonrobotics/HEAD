function TestClient(){
    let last_write = [];
    let last_error = [];
    let has_exited = false;

    let mock_write = function(msg){
        last_write.push(msg);
    }
    let mock_error = function(msg){
        last_error.push(msg);
    }
    let mock_exit = function(){
        has_exited = true;
    }

    let assertEquals = function(left, right, message) {
        if (!(left === right)) {
            message = (message || 'Assert failed') + '\nExpected:' + right + '\nFound:' + left;

            if (typeof Error !== 'undefined')
                throw new Error(message);

            throw message;
        }
    }
    let assert = function(bool, message) {
        if (!bool) {
            message = (message || 'Assert failed');

            if (typeof Error !== 'undefined')
                throw new Error(message);

            throw message;
        }
    }

    let make_client = function(){
        let client = new Client();
        client.write = mock_write;
        client.error = mock_error;
        client.exit = mock_exit;
        return client;
    }


    this.test_guid = function(){
        let uuid = guid();
        let set = {};
    }
    this.test_get = function(){
        let old_XMLHttpRequest = XMLHttpRequest;
        XMLHttpRequest = function(){//create mock REQ
            this.open_method = '';
            this.open_url = '';
            this.send_data = undefined;//default to non-null value
            this.open = function(method, url, async){
                this.open_method = method;
                this.open_url = url;
            }
            this.send = function(data){
                this.send_data = data;
            }
        };

        let params = {
            'str': 'foo bar baz\n\t\\\"',
            'num': 0123456789,
            'und': undefined,
        };
        let result = get('example.com',params);
        assertEquals(result.open_method, 'GET');
        assertEquals(result.open_url, 'example.com?str=foo%20bar%20baz%0A%09%5C%22&num=123456789');
        assertEquals(result.send_data, null);

        result = get('example.com?foo=bar',params);
        assertEquals(result.open_method, 'GET');
        assertEquals(result.open_url, 'example.com?foo=bar&str=foo%20bar%20baz%0A%09%5C%22&num=123456789');
        assertEquals(result.send_data, null);

        XMLHttpRequest = old_XMLHttpRequest;
    }
    this.test_connection = function(){
        const vanilla_client = new Client();
        let params = {
            'botid': vanilla_client.get_bot_id(),
            'question': 'How are you?',
            'session': vanilla_client.get_SESSION(),
            'lang': vanilla_client.get_lang(),
            'Auth': KEY
        };
        let url = vanilla_client.get_chatbot_url() + '/chat';
        let response = get(url,params);
        let text = response.responseText;
        let status_code = response.status;
        let json;
        try{
            json = JSON.parse(text);
        }catch(e){
            assert(false," Invalid JSON: " + text);
        }

        assertEquals(status_code, 200, " Server is not running.");
        assertEquals(json['ret'],0, "Response: " + JSON.stringify(json['response']));

        url = 'brokenurl.com';
        response = get(url,params);//Should print GET 404
        assertEquals(response.status, 404);

        params = {
            'botid': vanilla_client.get_bot_id(),
            'question': 'How are you?',
            'session': vanilla_client.get_SESSION(),
            'lang': vanilla_client.get_lang(),
            'Auth': 'not a key'
        };
        url = vanilla_client.get_chatbot_url() + '/chat';
        response = get(url,params);
        text = response.responseText;
        json = JSON.parse(text);
        assertEquals(response.status, 200);
        assertEquals(json['ret'],401);
    }
    this.test_ask = function(){
        let client = make_client();

        let old_get = get;//Mock the get function
        let url_in, params_in;
        get = function(url, params){
            url_in = url;
            params_in = params;
            return old_get(url, params);
        }

        const question = "Hi";
        const result = client.ask(question);
        assertEquals(url_in, 'http://localhost:8001/v1/chat');
        assertEquals(params_in.question, question);
    }

    this.test_list_chatbot = function(){
    }

    this.test_default = function(){
        let client = make_client();
        const msg = "Foo bar";
        client.default("Foo bar");
        assert(last_write[0].indexOf(client.get_bot_id()) !== -1);
        assertEquals(last_error.length,0);

        last_write = [];

        client.default();
        assertEquals(last_write.length,0);
        assertEquals(last_error.length,0);
    }

    this.test_do_list = function(){
        let client = make_client();
        let old_list_chatbot = client.list_chatbot;

        client.list_chatbot = function(line){
            return [
                ['foo',0.3],
                ['food',0.5],
                [client.get_bot_id() + 'foo', 0.75],
                [client.get_bot_id(), 0.141592654],
                ['foo' + client.get_bot_id(), 0.07]
                    ];
        }

        client.do_list("yes");
        client.do_list("no");
        assertEquals(last_write[0],last_write[1]);
        const expected = "foo: 0.3\n"+
            "food: 0.5\n"+
            "sophiafoo: 0.75\n"+
            "[sophia]: 0.141592654\n"+
            "foosophia: 0.07\n";
        assertEquals(last_write[0], expected);

        last_write = [];

        let line_input;
        client.list_chatbot = function(line){
            line_input = line;
            if(line === undefined){
                return [['foo',0.3]];
            }else{
                return [['bar',0.3]];
            }
        }

        client.do_list("foo");
        assertEquals(line_input, client.get_bot_id());
        client.do_list("all");
        assertEquals(line_input, undefined);
        assert(last_write[0] !== last_write[1]);

        client.list_chatbot = old_list_chatbot;
        last_write = [];
    }
    this.test_do_select = function(){
        let client = make_client();
        let old_list_chatbot = client.list_chatbot;

        client.list_chatbot = function(){
            return [["foo",0.3],
                   ["bar",0.2],
                   ["baz",0.1]];
        }

        client.do_select("bath");
        client.do_select("bazz");
        client.do_select("bar");

        assertEquals(last_write[0], "No such chatbot bath");
        assertEquals(last_write[1], "No such chatbot bazz");
        assertEquals(last_write[2], "Select chatbot bar");
        assertEquals(last_write.length, 3);

        last_write = [];
    }
    this.test_do_conn = function(){
        let client = make_client();

        client.do_conn();
        assert(last_write[0].indexOf(client.get_chatbot_url()) !== -1);
        assert(last_write[0].indexOf("foo.com") === -1);

        client.do_conn("foo.com");
        client.do_conn();
        assert(last_write[1].indexOf("foo.com") !== -1);
        assert(last_write[2].indexOf("foo.com") !== -1);
        assertEquals(client.get_chatbot_url(),"foo.com");

        last_write = [];
    }

    this.test_do_ip = function(){
        let client = make_client();

        client.do_ip();
        assert(last_write[0].indexOf(client.get_chatbot_ip()) !== -1);
        assert(last_write[0].indexOf("foo.com") === -1);

        client.do_ip("foo.com");
        client.do_ip();
        assertEquals(last_write[1],"ip is now foo.com");
        assertEquals(last_write[2],"url is now http://foo.com:" + client.get_chatbot_port() + "/" + VERSION);
        assert(last_write[3].indexOf("foo.com") !== -1);
        assertEquals(client.get_chatbot_ip(),"foo.com");
        assertEquals(client.get_chatbot_url(), 'http://foo.com:' + client.get_chatbot_port() + '/' + VERSION);

        client.do_conn("foo.com");

        assertEquals(client.get_chatbot_ip(),undefined);

        last_write = [];
    }
    this.test_do_port = function(){
        let client = make_client();

        client.do_port();
        assert(last_write[0].indexOf(client.get_chatbot_port()) !== -1);
        assert(last_write[0].indexOf("23840") === -1);

        client.do_port("23840");
        client.do_port();
        assert(last_write[1].indexOf("23840") !== -1);
        assertEquals(last_write[1],"port is now 23840");
        assertEquals(last_write[2],"url is now http://" + client.get_chatbot_ip() + ":23840/" + VERSION);
        assert(last_write[3].indexOf("23840") !== -1);
        assertEquals(client.get_chatbot_url(), 'http://' + client.get_chatbot_ip() + ':23840' + '/' + VERSION);

        client.do_conn("foo.com");

        assertEquals(client.get_chatbot_port(),undefined);

        last_write = [];
    }
    
    this.test_do_lang = function(){}
    this.test_do_c = function(){}
    this.test_do_rw = function(){}

    this.run_tests = function(){
        let self = this;
        for(prop in self){
            if(prop.indexOf('test_') == -1){
                continue;
            }
            const fn = self[prop];
            if(typeof(fn) === 'function'){
                try{
                    fn();
                    const msg = prop + ': pass';
                    write(msg);
                }catch(e){
                    const msg = prop + ': ' + e.stack;
                    write_error(msg);
                }
            }
        }
    }
}

let test_client = new TestClient();
test_client.run_tests();
