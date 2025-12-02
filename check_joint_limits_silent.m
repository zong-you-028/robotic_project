% check_joint_limits_silent.m
function in_range = check_joint_limits_silent(joint_vars, limits)
    % This function checks if joint variables are within limits
    % without printing any messages to the console.
    
    % Check lower bound (Note: limits(:, 1)' transposes 6x1 to 1x6)
    lower_check = (joint_vars >= limits(:, 1)');
    
    % Check upper bound (Note: limits(:, 2)' transposes 6x1 to 1x6)
    upper_check = (joint_vars <= limits(:, 2)');
    
    % Combine checks (logical AND)
    % The result is a 1x6 logical array [1, 1, 0, ...]
    in_range = lower_check & upper_check;
end