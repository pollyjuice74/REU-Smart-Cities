% half_duplex_controller.m – Alternate TX/RX in time
clc;
fprintf("Starting Half-Duplex loop...\n");

while true
    % 1. Transmit for X seconds
    fprintf("TX mode...\n");
    run transmitter/tx_main.m
    pause(0.5); % or use timer, async flags

    % 2. Listen for Y seconds
    fprintf("RX mode...\n");
    run receiver/rx_main.m
    pause(1.0); % Adjust for latency/processing

    % 3. Optional break condition
    user_input = input("Continue? [Y/n]:", 's');
    if strcmpi(user_input, 'n'), break; end
end
