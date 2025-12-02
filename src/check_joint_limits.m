% check_joint_limits.m
function is_valid = check_joint_limits(joint_vars, limits)
    is_valid = true;
    for i = 1:6
        if joint_vars(i) < limits(i, 1) || joint_vars(i) > limits(i, 2)
            if i == 3
                fprintf('d%d (%.2f) is out of range! (%.0f ~ %.0f)\n', i, joint_vars(i), limits(i,1), limits(i,2));
            else
                fprintf('theta%d (%.2f) is out of range! (%.0f ~ %.0f)\n', i, joint_vars(i), limits(i,1), limits(i,2));
            end
            is_valid = false;
        end
    end
end

% check_joint_limits_silent.m
function in_range = check_joint_limits_silent(joint_vars, limits)
    in_range = (joint_vars >= limits(:, 1)') & (joint_vars <= limits(:, 2)');
end