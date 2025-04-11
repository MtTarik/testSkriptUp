# Adaptive Kalman Altitude Filter (Lua)

This is a simple experimental altitude estimator based on a 2D Kalman filter (altitude + vertical speed) written in Lua.

The simulation runs for 10 minutes and models realistic drone flight behavior, including:
- Takeoff, climb, cruise, turbulence, descent, and landing phases
- Sensor switching between Baro, GPS, LIDAR, and RADAR
- Adaptive measurement noise (R) and process noise (Q)
- Basic GPS spoofing simulation and tilt compensation
- Outlier rejection and per-sensor confidence estimation

This project was built to test and visualize Kalman filter logic in real-like flight conditions before integrating into autopilot systems.

### Run

Just run the Lua script using any Lua 5.1+ interpreter.

### Note

This is an offline simulation for algorithm validation purposes only.  
Not intended for real-time onboard use (yet).

---

Author: Taras S. (🛠 engineering + experimentation)
