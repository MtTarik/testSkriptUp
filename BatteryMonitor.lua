function update()
    local voltage = battery:voltage(0)
    if voltage < 10.5 then
        gcs:send_text(6, string.format("LOW BATTERY: %.2f V", voltage))
    end
    return update, 2000
end
return update()
