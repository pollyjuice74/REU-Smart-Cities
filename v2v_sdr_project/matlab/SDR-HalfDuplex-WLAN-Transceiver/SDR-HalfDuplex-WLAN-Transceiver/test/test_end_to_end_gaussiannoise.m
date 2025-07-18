% Test end-to-end transmission over Gaussian noise channel
disp("Running end-to-end test: Gaussian Noise");

% Set config to add moderate noise
config.waveform_config.channel = 'GaussianNoise';
config.waveform_config.SNR = 30;  % dB
config.waveform_config.osf = 1.5;

% Save config to workspace
assignin('base', 'config', config);

% Run main
main;

% === ASSERTIONS ===
if exist('rxDataOrdered','var') && exist('txDataBits','var')
    testBits = txDataBits(1:length(rxDataOrdered));
    errors = sum(rxDataOrdered ~= testBits);
    fprintf("Total Bit Errors: %d\n", errors);
    assert(errors < 100, "Test failed: Too many bit errors under Gaussian Noise.");
    disp("✅ GaussianNoise test passed: BER acceptable");
else
    error("Test failed: No received data found");
end
