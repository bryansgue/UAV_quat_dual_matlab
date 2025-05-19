function error = quaternionError(q1, q2)
    % Normalize the quaternions
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);
    
    % Calculate the dot product between normalized quaternions
    dot_product = dot(q1, q2);
    
    % Calculate the error (angle difference) between quaternions
    error = acos(2 * dot_product^2 - 1);
end