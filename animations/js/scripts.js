$(function() {
	// ROS object to connect to ROS

	var ros = new ROSLIB.Ros();
  var animations = [];
	var cmdAnimations;
	var lastAnim = 'happy-1'
	// functions
  // Starts Animations mode in blender
	function startBlenderMode(){
		var cmdBlender = new ROSLIB.Topic({
		  ros : ros,
		  name : '/cmd_blendermode',
		  messageType : 'std_msgs/String'
		});
		var msg = new ROSLIB.Message({
		  data: 'Animations'
		});
    cmdBlender.publish(msg);
    	var cmdBllink = new ROSLIB.Topic({
		  ros : ros,
		  name : '/dmitry/cmd_blink',
		  messageType : 'std_msgs/String'
		});
		var msg = new ROSLIB.Message({
		  data: 'dmitry:stop'
		});
        cmdBllink.publish(msg);
    	var cmdTree = new ROSLIB.Topic({
		  ros : ros,
		  name : '/dmitry/behavior_switch',
		  messageType : 'std_msgs/String'
		});
		var msg = new ROSLIB.Message({
		  data: 'btree_off'
		});
        cmdTree.publish(msg);
	}

  // Get animations 
  function getAnimations(){
		console.log('anims');
		var anims = new ROSLIB.Topic({
		  ros : ros,
		  name : '/animations_list',
		  messageType : 'robo_blender/animations_list'
		});	
    anims.subscribe(function(msg){
			animations= msg.actions;
			anims.unsubscribe();
		});	
  }

	function addButtons(){
		cmdAnimations = new ROSLIB.Topic({
		  ros : ros,
		  name : '/cmd_animations',
		  messageType : 'std_msgs/String'
		});

		$('.emotion').click(function(){
			var data = $(this).data('animation');
			console.log(data);
			var msg = new ROSLIB.Message({
				data: 'play:'+data
			});
			lastAnim = data;
			cmdAnimations.publish(msg);
		});
		$('.command').click(function(){
			var data = $(this).data('cmd');
			console.log(data);
			if (data == 'play'){
				data = data+':'+lastAnim
			}
			var msg = new ROSLIB.Message({
				data: data
			});
			cmdAnimations.publish(msg);
		});
		$('#cmd_stop').click();

	}

  // Find out exactly when we made a connection.
  ros.on('connection', function() {
    console.log('Connection made!');
 		startBlenderMode();
		setTimeout(function(){addButtons()},1000);
		
  });

	var loopOn = 0;

	$('#demo-loop').click(function() {
		  $(this).find('.btn').toggleClass('active');  
		  
		  if ($(this).find('.btn-primary').size()>0) {
		  	$(this).find('.btn').toggleClass('btn-primary');
		  }
		  if ($(this).find('.btn-danger').size()>0) {
		  	$(this).find('.btn').toggleClass('btn-danger');
		  }
		  if ($(this).find('.btn-success').size()>0) {
		  	$(this).find('.btn').toggleClass('btn-success');
		  }
		  if ($(this).find('.btn-info').size()>0) {
		  	$(this).find('.btn').toggleClass('btn-info');
		  }
		  $(this).find('.btn').toggleClass('btn-default');
			// process the logic
			loopOn = $('#loop-on.active').length;
			if (loopOn){
				$('.optional').hide();
			}else{
				$('.optional').show();
			}

		
	});
  // Create a connection to the rosbridge WebSocket server.
  ros.connect("wss://" + document.domain + ":9090");
  //Demo loop
	jQuery.fn.random = function() {
		  var randomIndex = Math.floor(Math.random() * this.length);  
		  return jQuery(this[randomIndex]);
	};
	var demo = setInterval(function(){
		if (loopOn>0){
			$('.emotion').random().click();
		}
	},1000);
	

}); 
