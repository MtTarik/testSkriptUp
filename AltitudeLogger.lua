function fkm()
    local all = ahrs:altitude_relative()
    gcs:send_text(6, string.format("REL ALT: %.2f м", alt))
    return log_altitude, 1000
    
end

function log_altitude()
    local alt = ahrs:altitude_relative()
    gcs:send_text(6, string.format("REL AL: %.2f m", alt))
    return log_altitude, 5000
    
end
function log_mmdl ()
    local alt = ahe                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    
end


-- function log_altitude()
--     local alt = ahrs:altitude_relative()
--     gcs:send_text(6, string.format("REL ALT: %.2f м", alt))
--     return log_altitude, 5000 
-- end

return log_altitude()


