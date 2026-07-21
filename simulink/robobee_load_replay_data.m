function output = robobee_load_replay_data(logFile, outputType)
%ROBOBEE_LOAD_REPLAY_DATA Load synchronized thrust and torque replay data.
%   COMMANDS = ROBOBEE_LOAD_REPLAY_DATA(LOGFILE, 'commands') returns an
%   N-by-5 array in From Workspace format. The first column is elapsed time,
%   followed by desired thrust, roll torque, pitch torque, and yaw torque.
%
%   SAMPLE_TIME = ROBOBEE_LOAD_REPLAY_DATA(LOGFILE, 'sample_time') returns
%   the physical timestep encoded by time_vicon.

if nargin ~= 2
    error('RoboBee:Replay:InvalidInputCount', ...
        'Specify a log file and either ''commands'' or ''sample_time''.');
end

logFile = char(string(logFile));
if ~isfile(logFile)
    localFile = fullfile(fileparts(mfilename('fullpath')), logFile);
    if isfile(localFile)
        logFile = localFile;
    else
        error('RoboBee:Replay:FileNotFound', ...
            'RoboBee replay log not found: %s', logFile);
    end
end

fileInfo = dir(logFile);
canonicalFile = fullfile(fileInfo.folder, fileInfo.name);

persistent cachedFile cachedTimestamp cachedBytes cachedCommands cachedSampleTime
cacheIsValid = strcmp(cachedFile, canonicalFile) ...
    && isequal(cachedTimestamp, fileInfo.datenum) ...
    && isequal(cachedBytes, fileInfo.bytes);

if ~cacheIsValid
    requiredVariables = { ...
        'time_vicon', ...
        'thrust_desired_output', ...
        'roll_desired_output', ...
        'pitch_desired_output', ...
        'yaw_desired_output'};
    logData = load(canonicalFile, requiredVariables{:});

    missingVariables = setdiff(requiredVariables, fieldnames(logData));
    if ~isempty(missingVariables)
        error('RoboBee:Replay:MissingVariables', ...
            'Replay log %s is missing: %s', canonicalFile, ...
            strjoin(missingVariables, ', '));
    end

    time = double(logData.time_vicon(:));
    commands = [ ...
        1.1*double(logData.thrust_desired_output(:)), ...
        1.2*double(logData.roll_desired_output(:)), ...
        double(logData.pitch_desired_output(:)), ...
        double(logData.yaw_desired_output(:))];

    % Drop the leading zero-command preamble (the logs record ~2 s of zero
    % drive before the flight begins). Replay then starts at the first actual
    % command with the reconstructed clock re-zeroed to t = 0 below, removing
    % the initial dead time instead of replaying it.
    isZeroCommandRow = all(commands == 0, 2);
    firstActiveSample = find(~isZeroCommandRow, 1);
    if isempty(firstActiveSample)
        error('RoboBee:Replay:AllZeroCommands', ...
            'Replay log %s contains no nonzero commands.', canonicalFile);
    end
    time = time(firstActiveSample:end);
    commands = commands(firstActiveSample:end, :);

    if numel(time) < 2
        error('RoboBee:Replay:InsufficientSamples', ...
            'Replay log %s must contain at least two samples.', canonicalFile);
    end
    if size(commands, 1) ~= numel(time)
        error('RoboBee:Replay:LengthMismatch', ...
            'Replay timestamps and desired command arrays must have equal lengths.');
    end
    if any(~isfinite(time)) || any(~isfinite(commands), 'all')
        error('RoboBee:Replay:NonfiniteData', ...
            'Replay timestamps and desired commands must be finite.');
    end

    timeDelta = diff(time);
    if any(timeDelta <= 0)
        error('RoboBee:Replay:NonmonotonicTime', ...
            'Replay timestamps must be strictly increasing.');
    end

    sampleTime = round(median(timeDelta), 12, 'significant');
    uniformTolerance = max(1e-12, 1e-6 * sampleTime);
    if any(abs(timeDelta - sampleTime) > uniformTolerance)
        error('RoboBee:Replay:NonuniformTime', ...
            'Replay timestamps must use one physical timestep.');
    end

    % Reconstruct the verified uniform clock to avoid accumulated timestamp
    % roundoff changing which sample a fixed-step solver selects.
    replayTime = (0:numel(time) - 1)' * sampleTime;

    cachedFile = canonicalFile;
    cachedTimestamp = fileInfo.datenum;
    cachedBytes = fileInfo.bytes;
    cachedCommands = [replayTime, commands];
    cachedSampleTime = sampleTime;
end

switch lower(char(string(outputType)))
    case 'commands'
        output = cachedCommands;
    case 'sample_time'
        output = cachedSampleTime;
    otherwise
        error('RoboBee:Replay:InvalidOutputType', ...
            'Output type must be ''commands'' or ''sample_time''.');
end
end