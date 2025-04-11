local estimate = 0
local error_estimate = 1
local error_measure = 1
local q = 0.05

function test(z)
    local k = error_estimate /(error_estimate + error_measure)
    error_estimate = estimate + k *(z - estimate) 
    error_estimate = (1 - k) * error_estimate + math.abs(estimate) * q
    return estimate
end
function update()
    
    local raw =  ahrs:relalt()
    local filtered = kalman(raw)
    
    gcs:send_text(6, string.format("ALT_RAW: %.2f | KALMAN: %.2f", raw, filtered))
    
    return estimate,1000  

    
end
return update,1000



