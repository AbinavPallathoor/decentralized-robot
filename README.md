<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Omni-Base README</title>
<style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif; line-height: 1.6; color: #333; max-width: 800px; margin: 0 auto; padding: 20px; }
    h1 { border-bottom: 1px solid #eaecef; padding-bottom: 10px; color: #24292e; }
    h2 { border-bottom: 1px solid #eaecef; padding-bottom: 10px; margin-top: 30px; color: #24292e; }
    code { background-color: #f6f8fa; padding: 2px 4px; border-radius: 3px; font-family: monospace; }
    pre { background-color: #f6f8fa; padding: 16px; border-radius: 6px; overflow: auto; }
    blockquote { border-left: 4px solid #dfe2e5; padding: 10px 20px; color: #6a737d; margin: 0; background: #fcfcfc; }
    .emoji { font-size: 1.2em; }
</style>
</head>
<body>

<h1>Omni-Base ROS 2 Autonomous Navigation</h1>

<p>A complete, custom-built autonomous navigation stack for an omni-directional (holonomic) robot in ROS 2 Humble.</p>

<p>This project bypasses the standard Nav2 behavior trees and local planners in favor of a fully custom Python-based <strong>A* Global Planner</strong> (with obstacle inflation) and a custom <strong>Unified Holonomic Navigator</strong> (with Artificial Potential Field obstacle avoidance). It utilizes <code>slam_toolbox</code> for room mapping, <code>nav2_amcl</code> for precise localization, and OpenCV for autonomous frontier exploration.</p>

<h2>🛠️ Prerequisites & Dependencies</h2>
<p>This project is built for <strong>ROS 2 Humble</strong> running on a Raspberry Pi (or similar SBC) interacting with an STM32 base controller.</p>
<pre><code>sudo apt update
sudo apt install ros-humble-slam-toolbox                  ros-humble-teleop-twist-keyboard                  ros-humble-nav2-amcl                  ros-humble-nav2-map-server                  ros-humble-nav2-lifecycle-manager                  python3-opencv</code></pre>

<h2>🏗️ Installation & Build</h2>
<ol>
    <li>Clone this package into your ROS 2 workspace (e.g., <code>~/ros2_ws/src/</code>).</li>
    <li>Build the workspace:</li>
</ol>
<pre><code>cd ~/ros2_ws
colcon build --packages-select omni_base
source install/setup.bash</code></pre>

<h2>🗺️ Phase 1: Fully Autonomous Mapping</h2>
<p>You can let the robot completely map a room on its own using the Frontier Explorer.</p>
<ul>
    <li><strong>Launch the Autonomous Mapping Stack:</strong>
        <pre><code>ros2 launch omni_base autonomous_mapping.launch.py</code></pre>
    </li>
    <li><strong>Save the Map:</strong> (Do not close the launch file until saved!)
        <pre><code>ros2 run nav2_map_server map_saver_cli -f ~/my_map</code></pre>
    </li>
</ul>

<h2>🚀 Phase 2: Manual Waypoint Navigation</h2>
<h3>1. Launch Localization (AMCL)</h3>
<pre><code>ros2 launch omni_base localization.launch.py</code></pre>
<blockquote><strong>Note:</strong> Open Foxglove Studio, use the <em>Publish 2D pose estimate</em> (/initialpose) tool, and click on the map to initialize AMCL.</blockquote>

<h3>2. Run the Custom A* Planner</h3>
<pre><code>ros2 run omni_base astar_planner</code></pre>

<h3>3. Run the Unified Holonomic Navigator</h3>
<pre><code>ros2 run omni_base holonomic_navigator</code></pre>

<h2>🎮 Interacting via Foxglove Studio</h2>
<ul>
    <li><strong>Set Initial Location:</strong> Use the <em>Publish 2D pose estimate</em> tool on the <code>/initialpose</code> topic.</li>
    <li><strong>Send a Navigation Goal:</strong> Use the <em>Publish 2D pose</em> tool on the <code>/move_base_simple/goal</code> topic.</li>
    <li><strong>Visualize the Path:</strong> Subscribe to the <code>/astar_rigid_path</code> topic to see the route in real-time.</li>
</ul>

<h2>⚙️ Customization & Tuning</h2>
<ul>
    <li><strong>Obstacle Inflation:</strong> Edit <code>self.inflation_radius</code> in <code>astar_planner.py</code>.</li>
    <li><strong>Holonomic Navigator Tuning:</strong> Open <code>holonomic_navigator.py</code> to adjust:
        <ul>
            <li><code>max_speed</code> / <code>max_yaw_rate</code>: Robot speed limits.</li>
            <li><code>lookahead_dist</code>: Path smoothing.</li>
            <li><code>accel_alpha</code>: Low-pass filter for acceleration.</li>
            <li><code>danger_radius</code>: Obstacle avoidance sensitivity.</li>
        </ul>
    </li>
    <li><strong>AMCL Omni-Model:</strong> Adjust <code>alpha1</code> through <code>alpha5</code> in <code>localization.launch.py</code> to tune sensor trust.</li>
</ul>

</body>
</html>
