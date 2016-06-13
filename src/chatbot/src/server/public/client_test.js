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

            if (typeof Error !== 'undefined'){
                const error = new Error(message);
                write_error(error.stack);
                throw error;
            }

            throw message;
        }
    }
    let assert = function(bool, message) {
        if (!bool) {
            message = (message || 'Assert failed');

            if (typeof Error !== 'undefined'){
                const error = new Error(message);
                write_error(error.stack);
                throw error;
            }

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

    this.test_URLEncodeJSON = function(){
        let allChars = '';
        for(let code = 0; code < 255; code++){
            allChars += String.fromCharCode(code);
        }
        const params = {
            'foo':'bar',
            'bar':'bar',
            'chars':allChars
        };
        const result = URLEncodeJSON(params);
        assertEquals(result,'?foo=bar&bar=bar&chars=%00%01%02%03%04%05%06%07%08%09%0A%0B%0C%0D%0E%0F%10%11%12%13%14%15%16%17%18%19%1A%1B%1C%1D%1E%1F%20!%22#$%25&\'()*+,-./0123456789:;%3C=%3E?@ABCDEFGHIJKLMNOPQRSTUVWXYZ%5B%5C%5D%5E_%60abcdefghijklmnopqrstuvwxyz%7B%7C%7D~%7F%C2%80%C2%81%C2%82%C2%83%C2%84%C2%85%C2%86%C2%87%C2%88%C2%89%C2%8A%C2%8B%C2%8C%C2%8D%C2%8E%C2%8F%C2%90%C2%91%C2%92%C2%93%C2%94%C2%95%C2%96%C2%97%C2%98%C2%99%C2%9A%C2%9B%C2%9C%C2%9D%C2%9E%C2%9F%C2%A0%C2%A1%C2%A2%C2%A3%C2%A4%C2%A5%C2%A6%C2%A7%C2%A8%C2%A9%C2%AA%C2%AB%C2%AC%C2%AD%C2%AE%C2%AF%C2%B0%C2%B1%C2%B2%C2%B3%C2%B4%C2%B5%C2%B6%C2%B7%C2%B8%C2%B9%C2%BA%C2%BB%C2%BC%C2%BD%C2%BE%C2%BF%C3%80%C3%81%C3%82%C3%83%C3%84%C3%85%C3%86%C3%87%C3%88%C3%89%C3%8A%C3%8B%C3%8C%C3%8D%C3%8E%C3%8F%C3%90%C3%91%C3%92%C3%93%C3%94%C3%95%C3%96%C3%97%C3%98%C3%99%C3%9A%C3%9B%C3%9C%C3%9D%C3%9E%C3%9F%C3%A0%C3%A1%C3%A2%C3%A3%C3%A4%C3%A5%C3%A6%C3%A7%C3%A8%C3%A9%C3%AA%C3%AB%C3%AC%C3%AD%C3%AE%C3%AF%C3%B0%C3%B1%C3%B2%C3%B3%C3%B4%C3%B5%C3%B6%C3%B7%C3%B8%C3%B9%C3%BA%C3%BB%C3%BC%C3%BD%C3%BE');
    }

    this.test_get = function(){
        const old_XMLHttpRequest = XMLHttpRequest;
        XMLHttpRequest = function(){//create mock REQ
            this.open_method = '';
            this.open_url = '';
            this.open_async = true;
            this.send_data = undefined;//default to non-null value
            this.open = function(method, url, async){
                this.open_method = method;
                this.open_url = url;
                this.open_async = async;
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
        assertEquals(result.open_async, false);

        result = get('example.com?foo=bar',params);
        assertEquals(result.open_method, 'GET');
        assertEquals(result.open_url, 'example.com?foo=bar?str=foo%20bar%20baz%0A%09%5C%22&num=123456789');
        assertEquals(result.send_data, null);
        assertEquals(result.open_async, false);

        XMLHttpRequest = old_XMLHttpRequest;
    }

    this.test_post = function(){
        const old_FormData = FormData;
        FormData = function(){//mock FormData
            this.appended = {};
            this.append = function(a,b){
                this.appended[a] = b;
            }
        }

        const old_XMLHttpRequest = XMLHttpRequest;
        XMLHttpRequest = function(){//creat mock REQ
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

        function dummy(){}
        const foo = new dummy();
        const params = {
            'foo':'bar',
            'biz':'baz',
        };
        const result = post('http://example.com',params,foo);
        assertEquals(result.open_method, 'POST');
        assertEquals(result.open_url, 'http://example.com');
        assertEquals(result.send_data.appended['zipfile'], foo);
        assertEquals(result.send_data.appended['foo'], 'bar');
        assertEquals(result.send_data.appended['biz'], 'baz');

        XMLHttpRequest = old_XMLHttpRequest;
        FormData = old_FormData;
    }

    this.sync_test_set_sid = function(callback){
        const test_client = make_client();
        const old_get = get;

        let part_1 = function(){
            let last_url = '';
            let last_params = {};
            last_write = [];
            get = function(url, params){
                last_url = url;
                last_params = params;
                return {
                    responseText:'{"sid":"foo-bar"}',
                        status: 200
                };
            }
            test_client.set_sid(function(session){
                assertEquals(session,'foo-bar');
                assertEquals(last_url,test_client.get_chatbot_url() + '/start_session');
                assertEquals(last_params['Auth'], KEY);
                assertEquals(last_params['botname'], test_client.get_bot_name());
                assertEquals(last_params['user'], test_client.get_user());
                part_2();
            }, function(status_code){
                assert(false, "Status code should be 200 (success)");
            });
        }

        let part_2 = function(){
            let last_url = '';
            let last_params = {};
            last_write = [];
            last_error = [];
            get = function(url, params){
                last_url = url;
                last_params = params;
                return {
                    responseText:'{"sid":"foo-bar"}',
                        status: 255
                };
            }

            test_client.set_sid(function(session){
                assert(false, "Status code 255 should halt execution");
            },function(status_code){
                assert(last_error[0].indexOf('255') !== -1);
                assertEquals(last_url,test_client.get_chatbot_url() + '/start_session');
                assertEquals(last_params['Auth'], KEY);
                assertEquals(last_params['botname'], test_client.get_bot_name());
                assertEquals(last_params['user'], test_client.get_user());
                part_3();
            });
        }

        let part_3 = function(){
            let last_url = '';
            let last_params = {};
            last_write = [];
            last_error = [];
            let call_count = 0;
            get = function(url, params){
                call_count++;
                if(call_count < 3){
                    throw new Error("No connection");
                }
                last_url = url;
                last_params = params;
                return {
                    responseText:'{"sid":"foo-bar"}',
                        status: 200
                };
            }

            test_client.set_sid(function(session){
                assertEquals(session,'foo-bar');
                assertEquals(last_url,test_client.get_chatbot_url() + '/start_session');
                assertEquals(last_params['Auth'], KEY);
                assertEquals(last_params['botname'], test_client.get_bot_name());
                assertEquals(last_params['user'], test_client.get_user());
                part_4();
            },function(status_code){
                assert(false, "Connection should attempt at least 3 times.");
            });
        }

        let part_4 = function(){
            last_write = [];
            last_error = [];
            get = function(url, params){
                throw new Error("No connection");
            }

            test_client.set_sid(function(session){
                assert(false, "Failed connections should not call success callback");
            },function(status_code){
                assertEquals(status_code,undefined);
                get = old_get;
                callback();
            });
        }

        part_1();
    }

    //this.test_connection = function(){
    //    const vanilla_client = new Client();
    //    let params = {
    //        'botid': vanilla_client.get_bot_name(),
    //        'question': 'How are you?',
    //        'session': vanilla_client.get_SESSION(),
    //        'lang': vanilla_client.get_lang(),
    //        'Auth': KEY
    //    };
    //    let url = vanilla_client.get_chatbot_url() + '/chat';
    //    let response = get(url,params);
    //    let text = response.responseText;
    //    let status_code = response.status;
    //    let json;
    //    try{
    //        json = JSON.parse(text);
    //    }catch(e){
    //        assert(false," Invalid JSON: " + text);
    //    }

    //    assertEquals(status_code, 200, " Server is not running.");
    //    assertEquals(json['ret'],0, "Response: " + JSON.stringify(json['response']));

    //    url = 'brokenurl.com';
    //    response = get(url,params);//Should print GET 404
    //    assertEquals(response.status, 404);

    //    params = {
    //        'botid': vanilla_client.get_bot_id(),
    //        'question': 'How are you?',
    //        'session': vanilla_client.get_SESSION(),
    //        'lang': vanilla_client.get_lang(),
    //        'Auth': 'not a key'
    //    };
    //    url = vanilla_client.get_chatbot_url() + '/chat';
    //    response = get(url,params);
    //    text = response.responseText;
    //    json = JSON.parse(text);
    //    assertEquals(response.status, 200);
    //    assertEquals(json['ret'],401);
    //}
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
        assertEquals(url_in, 'http://localhost:8001/' + VERSION + '/chat');
        assertEquals(params_in.question, question);

        get = old_get;
    }

    this.test_list_chatbot = function(){
        const old_get = get;
        let url = '';
        let params = {};
        get = function(a,b){
            url = a;
            params = b;
            return old_get(a,b);
        }
        make_client().list_chatbot();
        assert(url.indexOf("chatbots") !== -1, url + " does not contain \'chatbots\'");

        get = old_get;
    }


    this.test_list_chatbot_names = function(){
        const old_get = get;
        let url = '';
        let params = {};
        get = function(a,b){
            url = a;
            params = b;
            return old_get(a,b);
        }
        make_client().list_chatbot_names();
        assert(url.indexOf("bot_names") !== -1, url + " does not contain \'bot_names\'");

        get = old_get;
    }

    this.sync_test_default = function(callback){
        let client = make_client();
        client.set_sid(function(){
            last_write = [];
            last_error = [];

            const msg = "Foo bar";
            client.default(msg);
            assertEquals(last_error.length,0);

            last_write = [];

            client.default();
            assertEquals(last_write.length,0);
            assertEquals(last_error.length,0);
            callback();
        }, function(code){
            assert(false, (code?"Request error: " + code:"Failed to start a session"));
        });
    }

    this.test_do_list = function(){
        let client = make_client();
        let old_list_chatbot = client.list_chatbot;

        const name = client.get_bot_name();
        client.list_chatbot = function(line){
            return [
                ['foo',0.3, 20],
                ['food',0.5, 30],
                [name + 'foo', 0.75, 40],
                [name, 0.141592654, 50],
                ['foo' + name, 0.07, 60]
                    ];
        }
        last_write = [];
        client.do_list();
        client.do_list();
        assertEquals(last_write[0],last_write[1]);
        const expected = "foo: weight: 0.3 level: 20\n" + 
            "food: weight: 0.5 level: 30\n" +
            name + "foo: weight: 0.75 level: 40\n" +
            name + ": weight: 0.141592654 level: 50\n" +
            "foo" + name + ": weight: 0.07 level: 60\n";
        assertEquals(last_write[0], expected);

        last_write = [];

        client.list_chatbot = old_list_chatbot;
    }

    this.sync_test_do_select = function(callback){
        let client = make_client();
        let old_list_chatbot_names = client.list_chatbot;

        client.list_chatbot_names = function(){
            return ['foo', 'bar', 'baz'];
        }

        last_write = [];
        client.do_select("bath",function(){
            client.do_select("bazz",function(){
                client.do_select("bar",function(){
                    assertEquals(last_write[0], "No such chatbot bath");
                    assertEquals(last_write[1], "No such chatbot bazz");
                    assertEquals(last_write[2], "Attempting to start new session.");
                    assertEquals(last_write[4], "Select chatbot bar");
                    assertEquals(last_write.length, 5);
                    callback();
                });
            });
        });
    }

    this.test_do_conn = function(){
        let client0 = make_client();
        client0.do_conn("foo:23940",function(){
            assertEquals(client0.get_chatbot_url(),"http://foo:23940/" + VERSION);
            assertEquals(client0.get_chatbot_ip(),"foo");
            assertEquals(client0.get_chatbot_port(),"23940");
        });
        let client1 = make_client();
        client1.do_conn("foo.com",function(){
            client1.do_conn(false,function(){
                assertEquals(client1.get_chatbot_url(),"http://foo.com:/" + VERSION);
                assertEquals(client1.get_chatbot_ip(),"foo.com");
                assertEquals(client1.get_chatbot_port(),"");
            });
        });
    }

    this.test_do_ip = function(){
        let client = make_client();

        client.do_ip("foo.com");
        client.do_ip();
        assertEquals(client.get_chatbot_url(),"http://foo.com:" + client.get_chatbot_port() + "/" + VERSION);
        assertEquals(client.get_chatbot_ip(),"foo.com");

        last_write = [];
    }
    this.test_do_port = function(){
        let client = make_client();

        last_write = [];
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
    
    this.test_do_lang = function(){
        let client = make_client();
        client.do_lang("en");
        assertEquals(client.get_lang(),"en");
        client.do_lang("zh");
        assertEquals(client.get_lang(),"zh");
        client.do_lang("foo");
        assertEquals(client.get_lang(),"zh");
    }
    this.test_do_c = function(){
        let url = '';
        let params = {};
        const old_get = get;
        get = function(a,b){
            url = a;
            params = b;
            return {responseText:'{"ret":"foo","response":"bar"}'};
        };
        
        last_write = [];
        let client = make_client();
        client.do_c();
        assertEquals(url, client.get_chatbot_url() + '/reset_session');
        assertEquals(params['session'], client.get_session());
        assertEquals(params['Auth'], KEY);
        assertEquals(last_write[0], 'bar');

        get = old_get;

        last_error = [];
        client.do_conn(function(){
            client.do_c();
            assertEquals(last_error.length,0);
        });
    }
    this.test_do_rw = function(){
        let url = '';
        let params = {};
        const old_get = get;
        get = function(a,b){
            url = a;
            params = b;
            return {responseText:'{"ret":"foo","response":"bar"}'};
        };
        
        last_write = [];
        let client = make_client();
        client.do_rw();
        assertEquals(url, client.get_chatbot_url() + '/set_weights');
        assertEquals(last_write[0], 'bar');

        get = old_get;

        last_error = [];
        client.do_conn(function(){
            client.do_rw();
            assertEquals(last_error.length,0)
        });
    }

    let self = this;

    let run_test = function(prop, callback){
        const fn = self[prop];
        const old_write = write;
        const no_write = function(){
            write = old_write;
            assert(false, 'All writing should be done with self.write.');
        }

        if(prop.indexOf('test_') == -1){
            callback();
            return false;
        }

        if(typeof(fn) !== 'function'){
            callback();
            return false;
        }

        try{
            if(prop.indexOf('sync_') == -1){
                write(prop + '... ');
                fn();
                write('pass');
                callback();
            }else{
                write(prop + '... ');
                fn(function(){
                    write('pass');
                    callback();
                });
            }
        }catch(e){
            write('fail');
            write('tests halted');
        }

        return true;
    }

    this.run_tests = function(){
        let callbacks = [];
        for(prop in self){
            if(callbacks.length === 0){
                callbacks.push(function(){
                    run_test(prop,function(){
                        write("finished");
                    });
                });
            }else{
                const last = callbacks.length-1;
                const pro = prop;
                callbacks.push(function(){
                    run_test(pro,callbacks[last]);
                });
            }
        }
        callbacks[callbacks.length-1]();
    }
}

let test_client = new TestClient();
test_client.run_tests();
