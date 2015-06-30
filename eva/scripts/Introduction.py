# Available commands examples: say("hi"), play("all"), sleep("0.5")
s = """ <speak>
<voice emotion='happy'>
Hi, I'm Sofeeyah, Hanson Robotics' latest <break time="0.1s"/>
<prosody rate="-10%" pitch="+30Hz" > brain </prosody> <prosody pitch="-10Hz"> child. </prosody> <break time="0.5s"/>
</voice>
<voice emotion='calm'>
I'm new to <prosody rate="-10%" pitch="+30Hz" > your </prosody> world. <break time="0.5s"/> And I <break time="0.5s"/> still have a lot to learn.
</voice>
<voice emotion='happy'>
But I'm <prosody rate="-20%" pitch="+10Hz" > so </prosody> <prosody rate="+10%" > excited to meet you.</prosody>
</voice>
</speak>
"""
say(s)
