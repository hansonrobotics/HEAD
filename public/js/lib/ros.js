RosUI.ros = {
    config: {},
    topics: {},
    init: function (success) {
        RosUI.ros.connect(success);
    },
    connect: function (success) {
        //Connect to rosbridge
        RosUI.ros.ros = new ROSLIB.Ros({
            url: RosUI.ros.rosUrl()
        }).on("connection", function (e) {
                // Get Robot name
                RosUI.api.getRobotName(function(val){
                    if (val){
                        // Finishe initializing UI
                        console.log('robot: '+val);
                        RosUI.robot = val
                        RosUI.ros.initTopics();
                        RosUI.ros.initServices();
                        success();                            
                    }else{
                        RosUI.ros.ros.close();
                    }
                });  
            }).on('connection', function () {
                $('#app-connecting').hide();
                $('#app-pages').fadeIn();
            }).on('close', function () {
                $('#notifications .label').hide();
                $('#app-connection-error').show();
                $('#app-title').html('');
            }).on('error', function (error) {
                $('#notifications .label').hide();
                $('#app-connection-error').show();
                $('#app-title').html('');
            });

    },
    initTopics: function () {
        RosUI.ros.topics = {
            cmdTree: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/behavior_switch',
                messageType: 'std_msgs/String'
            }),
            speech_topic: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/'+RosUI.robot + '/chatbot_speech',
                messageType: 'chatbot/ChatMessage'
            }),
            chat_responses: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/'+RosUI.robot + '/chatbot_responses',
                messageType: 'std_msgs/String'
            }),
            expression: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/'+RosUI.robot + '/make_face_expr',
                messageType: 'basic_head_api/MakeFaceExpr'
            }),
            pointHeadTopic: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/'+RosUI.robot + '/point_head',
                messageType: 'basic_head_api/PointHead'
            }),
            available_gestures: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/available_gestures'
            }),
            available_emotion_states: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/available_emotion_states'
            }),
            set_gesture: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/set_gesture',
                messageType: 'blender_api_msgs/SetGesture'
            }),
            set_emotion_state: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/set_emotion_state',
                messageType: 'blender_api_msgs/EmotionState'
            })
        };
    },
    initServices: function () {
        console.log('init services');
        RosUI.ros.services = {
            headPauMux: new ROSLIB.Service({
                ros: RosUI.ros.ros,
                name: '/'+RosUI.robot + '/head_pau_mux/select',
                serviceType: 'topic_tools/MuxSelect'
            }),
            neckPauMux: new ROSLIB.Service({
                ros: RosUI.ros.ros,
                name: '/'+RosUI.robot + '/neck_pau_mux/select',
                serviceType: 'topic_tools/MuxSelect'
            }),
            expressionList: new ROSLIB.Service({
                ros: RosUI.ros.ros,
                name: '/'+RosUI.robot + '/valid_face_exprs',
                serviceType: 'basic_head_api/ValidFaceExprs'
            })
        };
    },
    rosUrl: function () {
        if (window.location.protocol != "https:") {
            return "ws://" + document.domain + ":9090";
        } else {
            return "wss://" + document.domain + ":9094";
        }
    },
    getMotorConfig: function (name) {
        var motorConf = RosUI.ros.config.motors;

        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name == name)
                return motorConf[i];
        }
    }
};
