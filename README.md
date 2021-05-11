<!DOCTYPE html><html><head><meta charset="utf-8"><title>ARENA Robot Project.md</title><style></style></head><body id="preview">
<h1 class="code-line" data-line-start=0 data-line-end=1><a id="ARENA_Robot_Project_0"></a>ARENA Robot Project</h1>
<h2 class="code-line" data-line-start=1 data-line-end=2><a id="_Interact_with_Sphero_RVR_in_ARENA__1"></a><em>Interact with Sphero RVR in ARENA</em></h2>
<p class="has-line-data" data-line-start="3" data-line-end="5">This project demos the possibilities of interacting with a mobile robot in a ARENA<br>
AR scene.</p>
<h2 class="code-line" data-line-start=6 data-line-end=7><a id="Highlights_6"></a>Highlights</h2>
<ul>
<li class="has-line-data" data-line-start="8" data-line-end="9">The robot navigates without providing obstacle information beforehand</li>
<li class="has-line-data" data-line-start="9" data-line-end="10">Smooth navigation without pausing at each cell</li>
<li class="has-line-data" data-line-start="10" data-line-end="11">Shows live sensor information in the scene</li>
<li class="has-line-data" data-line-start="11" data-line-end="12">Shows waypoint after path is planned</li>
<li class="has-line-data" data-line-start="12" data-line-end="13">Waypoint disappears when RVR drives by or when path is replanned</li>
<li class="has-line-data" data-line-start="13" data-line-end="15">Shows obstacle when detected</li>
</ul>
<h2 class="code-line" data-line-start=15 data-line-end=16><a id="Demo_15"></a>Demo</h2>
<p class="has-line-data" data-line-start="16" data-line-end="17">Check out the video demo on YouTube by clicking the image below.</p>
<p class="has-line-data" data-line-start="18" data-line-end="19"><a href="https://www.youtube.com/watch?v=TdxpzFF49M0"><img src="https://img.youtube.com/vi/TdxpzFF49M0/default.jpg" alt="IMAGE ALT TEXT HERE"></a></p>
<ol>
<li class="has-line-data" data-line-start="19" data-line-end="20">The user clicks on the circle that represents the destination. The circle turns red.</li>
<li class="has-line-data" data-line-start="20" data-line-end="21">The robot plans the path and shows the waypoints to the destination and starts to drive</li>
<li class="has-line-data" data-line-start="21" data-line-end="22">When navigating, the robot will show the distance of an obstacle in front of it on the screen</li>
<li class="has-line-data" data-line-start="22" data-line-end="23">When an obstacle is too close, a box will appear at the location of the obstacle</li>
<li class="has-line-data" data-line-start="23" data-line-end="24">The robot will than replan the path and erase all the future path from the last path planning</li>
<li class="has-line-data" data-line-start="24" data-line-end="26">When arrived at the destination, the circle representing the destination turns back to white</li>
</ol>
<h2 class="code-line" data-line-start=26 data-line-end=27><a id="Hardware_26"></a>Hardware</h2>
<p class="has-line-data" data-line-start="28" data-line-end="29">These are the hardware used in this project</p>
<ul>
<li class="has-line-data" data-line-start="30" data-line-end="31"><a href="https://sphero.com/products/rvr">One Sphero RVR</a> - The mobile robot</li>
<li class="has-line-data" data-line-start="31" data-line-end="32"><a href="https://www.amazon.com/gp/product/B07B94C7KT/">One HC-SR04 sensor</a> - Ultrasonic sensor used to detect obstacle</li>
<li class="has-line-data" data-line-start="32" data-line-end="33"><a href="https://www.amazon.com/gp/product/B07D54XMFK/">One 330 Ohm resistor</a> - Used for RPi wiring</li>
<li class="has-line-data" data-line-start="33" data-line-end="34"><a href="https://www.amazon.com/gp/product/B07D54XMFK/">One 470 Ohm resistor</a> - Used for RPi wiring</li>
<li class="has-line-data" data-line-start="34" data-line-end="35"><a href="https://www.amazon.com/gp/product/B07PCJP9DY/">One 400-pin breadboard</a> - Used for RPi wiring</li>
<li class="has-line-data" data-line-start="35" data-line-end="36"><a href="https://www.amazon.com/Elegoo-EL-CP-004-Multicolored-Breadboard-arduino/dp/B01EV70C78/">Breadboard cables</a> - Used for RPi wiring</li>
<li class="has-line-data" data-line-start="36" data-line-end="37"><a href="https://www.amazon.com/CanaKit-Raspberry-Basic-Kit-8GB/dp/B08DJ9MLHV/">One RaspberryPi</a> - Connects to RVR and run the code</li>
<li class="has-line-data" data-line-start="37" data-line-end="38"><a href="https://www.amazon.com/gp/product/B00HS5BYVO/">12&quot;x12&quot; puzzle floor mats</a> - Create physical environment for RVR</li>
</ul>
<h2 class="code-line" data-line-start=40 data-line-end=41><a id="Setup_40"></a>Setup</h2>
<ul>
<li class="has-line-data" data-line-start="42" data-line-end="46">
<p class="has-line-data" data-line-start="42" data-line-end="45">Setup Raspberry Pi<br>
–Follow <a href="https://sdk.sphero.com/docs/getting_started/raspberry_pi/raspberry_pi_setup/">this official guide</a> of setting up RPi from Sphero SDK<br>
–After intallation of the OS, follow the steps here to <a href="https://piwithvic.com/raspberry-pi-expand-filesystem-micro-sd-card">expand the filesystem</a></p>
</li>
<li class="has-line-data" data-line-start="46" data-line-end="49">
<p class="has-line-data" data-line-start="46" data-line-end="48">Setup ultrasonic sensor<br>
–Follow <a href="https://sdk.sphero.com/docs/samples_content/raspberry_pi/python/ultrasonic_rvr_sample/">this example</a> to setup the wiring, use one sensor instead of two</p>
</li>
<li class="has-line-data" data-line-start="49" data-line-end="50">
<p class="has-line-data" data-line-start="49" data-line-end="50">Install ARENA-py Python repository on Raspberry Pi by running the command below</p>
</li>
</ul>
<pre><code class="has-line-data" data-line-start="51" data-line-end="53" class="language-sh">pip3 install arena-py
</code></pre>
<ul>
<li class="has-line-data" data-line-start="53" data-line-end="54">Put the sphero_sdk folder in the same folder as this project</li>
<li class="has-line-data" data-line-start="54" data-line-end="56">Modify this file /sphero_sdk/asyncio/client/toys/sphero_rvr_async.py<br>
At line 35:</li>
</ul>
<pre><code class="has-line-data" data-line-start="57" data-line-end="64" class="language-sh">FROM:
asyncio.get_event_loop().run_until_complete ( 
            self._check_rvr_fw() 
        )
To:
asyncio.ensure_future(self._check_rvr_fw())
</code></pre>
<h2 class="code-line" data-line-start=65 data-line-end=66><a id="Resources_65"></a>Resources</h2>
<ul>
<li class="has-line-data" data-line-start="66" data-line-end="70">Sphero Public SDK<br>
–<a href="https://sdk.sphero.com/docs/sdk_documentation/connection/">API documentation</a> - Not very useful, seems outdated<br>
–<a href="https://github.com/sphero-inc/sphero-sdk-raspberrypi-python">SDK GitHub repository</a> - Great examples in the getting_started folder<br>
–<a href="https://community.sphero.com/">The official forum</a> - Read <a href="https://community.sphero.com/t/programming-questions/829">this great discussion</a> to learn more about how RVR take inputs</li>
<li class="has-line-data" data-line-start="70" data-line-end="73">Path planning algorithm<br>
– <a href="https://www.redblobgames.com/pathfinding/a-star/introduction.html">Concept</a><br>
– <a href="https://www.redblobgames.com/pathfinding/a-star/implementation.html">Implementation examples</a></li>
<li class="has-line-data" data-line-start="73" data-line-end="75">Future work<br>
–<a href="https://www.intechopen.com/books/robot-control/occupancy-map-construction-for-indoor-robot-navigation#E5">Occupancy Map Construction for Indoor Robot Navigation</a></li>
</ul>
</body></html>
