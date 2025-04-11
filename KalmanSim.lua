
local Kalman = {}
Kalman.__index = Kalman

function Kalman:new()
    local self = setmetatable({}, Kalman)
    self.x = {0, 0}
    self.P = {
        {10.0, 0.0},
        {0.0, 10.0}
    }

    self.q_pos = 0.05
    self.q_vel = 0.01

    self.NUM_SENSORS = 4
    self.SENSOR_BARO = 1
    self.SENSOR_GPS = 2
    self.SENSOR_LIDAR = 3
    self.SENSOR_RADAR = 4

    self.r = {0.8, 1.2, 0.4, 0.6}
    self.entropy = {1, 1, 1, 1}
    self.confidence = {1, 1, 1, 1}
    return self
end

function Kalman:deg2rad(deg)
    return deg * math.pi / 180
end

function Kalman:tilt_compensated(z, sensor, pitch, roll)
    if sensor == self.SENSOR_LIDAR or sensor == self.SENSOR_RADAR then
        local correction = math.cos(self:deg2rad(pitch)) * math.cos(self:deg2rad(roll))
        return z * math.max(0.5, correction)
    else
        return z
    end
end

function Kalman:nonlinear_measurement(z, sensor)
    if sensor == self.SENSOR_GPS then
        return z + (math.random() * 5 - 2.5) -- GPS sp
    else
        return z
    end
end

function Kalman:update(z_raw, vz_meas, dt, sensor, pitch, roll, t)
    pitch = pitch or 0
    roll = roll or 0

    local z_tilt = self:tilt_compensated(z_raw, sensor, pitch, roll)
    local z_meas = self:nonlinear_measurement(z_tilt, sensor)
    local R = self.r[sensor]

    if self.x[1] == 0 and self.x[2] == 0 then
        self.x[1] = z_meas
        self.x[2] = vz_meas
        return
    end

    local Qz = self.q_pos * dt * dt
    local Qv = self.q_vel * dt

    local x0_pred = self.x[1] + self.x[2] * dt
    local x1_pred = self.x[2]

    local P00_pred = self.P[1][1] + dt * (self.P[2][1] + self.P[1][2]) + Qz
    local P01_pred = self.P[1][2] + self.P[2][2] * dt
    local P10_pred = self.P[2][1] + self.P[2][2] * dt
    local P11_pred = self.P[2][2] + Qv

    local innovation = x0_pred - z_meas
    local S = P00_pred + R

    if t > 1.0 and math.abs(innovation) > 4 * math.sqrt(S) then return end

    local K0 = P00_pred / S
    local K1 = P10_pred / S

    self.x[1] = x0_pred - K0 * innovation
    self.x[2] = x1_pred - K1 * innovation

    self.P[1][1] = (1 - K0) * P00_pred
    self.P[1][2] = (1 - K0) * P01_pred
    self.P[2][1] = -K1 * P00_pred + P10_pred
    self.P[2][2] = -K1 * P01_pred + P11_pred

    local entropy = math.abs(innovation) / math.sqrt(R)
    self.entropy[sensor] = 0.9 * self.entropy[sensor] + 0.1 * entropy
    self.confidence[sensor] = 1.0 / (1.0 + self.entropy[sensor])

    local alpha = 0.01
    local err2 = innovation * innovation
    local R_MIN = 0.01
    self.r[sensor] = math.max(R_MIN, (1 - alpha) * self.r[sensor] + alpha * err2)

    if self.entropy[sensor] < 0.5 then
        self.q_pos = self.q_pos * 0.98
        self.q_vel = self.q_vel * 0.98
    end
end

function Kalman:log(t, sensor)
    print(string.format("[t=%6.1f] ALT=%6.2f | VZ=%6.2f | SENSOR=%d", t, self.x[1], self.x[2], sensor))
end

-- ========  simulate_altitude ======== --

local tg = 1.12
local kalman = Kalman:new()
local t = 0
local dt = 0.1
local duration_sec = 600ї
local steps = duration_sec / dt

-- 
local function simulate_altitude(t)
    if t < 30 then  
        return 10 + 2 * t + math.random() * 0.5
    elseif t < 120 then 
        return 100 + math.sin(t * 0.1) * 5
    elseif t < 300 then 
        return 100 + math.sin(t * 0.5) * 10 + math.random() * 5
    elseif t < 500 then 
        return 100 + math.sin(t * 0.2)
    elseif t < 580 then  
        return 100 - (t - 500) * 1.2 + math.random() * 2
    else -- 
        return 5 + math.random() * 1.5
    end
end

local function simulate_pitch_roll(t)
    return math.sin(t) * 4, math.cos(t) * 4
end

local function choose_sensor(t)
    if t < 120 then return kalman.SENSOR_BARO
    elseif t < 240 then return kalman.SENSOR_GPS
    elseif t < 360 then return kalman.SENSOR_LIDAR
    else return kalman.SENSOR_RADAR end
end

for i = 1, steps do
    local alt = simulate_altitude(t)
    local vz = (simulate_altitude(t + dt) - alt) / dt
    local pitch, roll = simulate_pitch_roll(t)
    local sensor = choose_sensor(t)

    if math.random() < 0.05 then
        sensor = kalman.SENSOR_BARO
    end

    kalman:update(alt, vz, dt, sensor, pitch, roll, t)

    if i % 20 == 0 then 
        kalman:log(t, sensor)
    end

    t = t + dt
end
