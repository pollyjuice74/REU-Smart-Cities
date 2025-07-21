function safeRelease(dev)
    % Helper to release device safely (ignore errors)
    if ~isempty(dev) && isvalid(dev)
        try
            release(dev);
        catch
            % Ignore errors on release
        end
    end
end